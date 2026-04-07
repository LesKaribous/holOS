# holOS — Architecture (2026)

## Vue d'ensemble

Le robot est piloté par deux cerveaux qui collaborent via XBee ou USB-CDC :

```
┌─────────────────────────────────────────────────────────────────┐
│                  JETSON / PC  (software/)                        │
│                                                                 │
│  strategy/match.py   →  MotionService, SafetyService, …        │
│  Brain (brain.py)        (services/ — API identique sim/réel)  │
│  Transport abstrait  →  XBeeTransport (XBee radio)             │
│                          WiredTransport (USB-CDC direct)        │
│                          VirtualTransport (sim Windows)         │
│  run.py              →  Point d'entrée unifié (PC + Jetson)    │
│                          Auto-détecte la plateforme             │
└─────────────────────────────┬───────────────────────────────────┘
                              │  XBee 868 MHz ou USB-CDC
                              │  57600 baud (configurable)
                              │  protocole CRC8-SMBUS
                              │
┌─────────────────────────────▼───────────────────────────────────┐
│                      TEENSY 4.1  (firmware/teensy41/src/)        │
│                                                                 │
│  Cyclic 10µs   →  motion.step()   (stepper ISR)                │
│  Cyclic 1ms    →  motion.control() (PID)                        │
│  OS loop       →  tous les services                             │
│                                                                 │
│  JetsonBridge  →  dispatch commandes, télémétrie, watchdog     │
│  Safety        →  lidar + occupancy (local, jamais délégué)    │
│  BlockRegistry →  blocs C++ exécutables par Jetson ou local    │
│  RuntimeConfig →  config SD key=value (config.cfg)             │
│  StallDetector →  détection blocage par déplacement/temps      │
│  Fallback      →  si Jetson déconnecté → stratégie embarquée   │
└─────────────────────────────────────────────────────────────────┘
                              │  UART 31250 baud (Intercom)
┌─────────────────────────────▼───────────────────────────────────┐
│                      TEENSY 4.0  (lidar)                        │
│  Génère l'occupancy map → envoie au Teensy 4.1                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## Protocole de communication

Format des trames (identique au protocole Intercom existant) :

| Direction         | Format                           | Exemple                        |
|-------------------|----------------------------------|--------------------------------|
| Jetson → Teensy   | `<id>:<cmd>\|<crc8>\n`           | `42:go(500,300)\|123\n`         |
| Teensy → Jetson   | `r<id>:<response>\|<crc8>\n`     | `r42:ok\|45\n`                  |
| Télémétrie push   | `T:<chan> <data>\|<crc8>\n`       | `T:p 1234 679 1571\|78\n`      |
| Heartbeat         | `hb` / `pong`                    |                                |

CRC : **CRC8-SMBUS** (polynomial 0x07, init 0x00) — implémenté côté Python avec `crcmod`.

### Télémétrie (Teensy → Jetson) — Format compact

| Canal  | Alias | Format                                   | Fréquence par défaut |
|--------|-------|------------------------------------------|----------------------|
| `T:p`  | pos   | `x_mm y_mm theta_mrad`                   | 10 Hz (100 ms)       |
| `T:m`  | motion| `R tx ty dist feed%` / `I feed%` / `DONE:ok\|fail` | event + 10 Hz |
| `T:s`  | safety| `0` ou `1`                               | 2 Hz + on-change     |
| `T:c`  | chrono| `<elapsed_ms>`                           | 1 Hz (1000 ms)       |
| `T:od` | occ   | hex bytes (occupancy map compressée)     | 2 Hz (500 ms)        |
| `T:mask`| mask | `11110` (flags pos,motion,safety,chrono,occ) | on-change        |

Les fréquences sont configurables via RuntimeConfig (SD card) :
```
tel.pos_ms=100
tel.motion_ms=100
tel.safety_ms=500
tel.chrono_ms=1000
tel.occ_ms=500
```

Le format legacy (`TEL:pos:x=500,y=300,theta=1.57`) reste supporté côté Python pour rétrocompatibilité.

### Commandes supportées (Jetson → Teensy)

| Commande                         | Description                              | Réponse        |
|----------------------------------|------------------------------------------|----------------|
| `hb`                             | Heartbeat                                | `ok` immédiat  |
| `health`                         | Snapshot état services + stratégie       | `mo=1,...,strat=0\|1` |
| `match_start`                    | Démarrer le match (remote start)         | `ok` immédiat  |
| `match_stop`                     | Pause match + cancel motion              | `ok` immédiat  |
| `match_resume`                   | Reprendre le match                       | `ok` immédiat  |
| `go(<x>,<y>)`                    | Déplacement absolu                       | après fin move |
| `go_coc(<x>,<y>)`               | Déplacement + cancel on stall/collide    | après fin move |
| `goPolar(<angle>,<dist>)`        | Déplacement polaire                      | après fin move |
| `turn(<angle_deg>)`              | Rotation absolue                         | après fin move |
| `align(<side>,<angle>)`          | Aligner côté robot sur une orientation   | après fin move |
| `goAlign(<x>,<y>,<theta_deg>)`   | Déplacement + orientation finale         | après fin move |
| `cancel`                         | Annuler le mouvement courant             | `ok` immédiat  |
| `pause` / `resume`               | Pause / reprise motion                   | `ok` immédiat  |
| `setAbsPosition(<x>,<y>,<a>)`    | Forcer la position                       | `ok` immédiat  |
| `feed(<feedrate>)`               | Feedrate global [0.05..1.0]              | `ok` immédiat  |
| `elevator(<side>,<pose>)`        | Actionneur                               | `ok` immédiat  |
| `grab(<side>)` / `drop(<side>)`  | Prise / lâcher                           | `ok` immédiat  |
| `enable(SAFETY)` / `disable(…)`  | Safety                                   | `ok` immédiat  |
| `start`                          | Démarrer le match (terminal)             | `ok` immédiat  |
| `stop`                           | Arrêt robot                              | `ok` immédiat  |
| `fb(<id>)`                       | Déclencher un fallback (0-4)             | `ok` immédiat  |
| `blocks_list`                    | Lister les blocs C++ enregistrés         | `name=p,s,ms,done;...` |
| `run_block(<name>)`              | Exécuter un bloc C++ par nom             | `SUCCESS` / `FAILED` |
| `block_done(<name>)`             | Marquer un bloc comme terminé            | `ok` immédiat  |
| `cfg_list`                       | Lister la config runtime (SD)            | `key=val;...`  |
| `cfg_set(<key>,<val>)`           | Modifier une config runtime              | `ok` immédiat  |
| `cfg_save`                       | Sauvegarder config sur SD                | `ok` immédiat  |

---

## Structure des dossiers

```
holOS/
├── firmware/
│   └── teensy41/
│       └── src/
│           ├── config/
│           │   ├── env.h                 ← includes globaux + tous les singletons
│           │   ├── settings.h            ← constantes (baudrate, grille, PID, stall…)
│           │   ├── pin.h                 ← mapping pins
│           │   ├── poi.h                 ← points d'intérêt
│           │   ├── runtime_config.h/.cpp ← config SD key=value (/config.cfg)
│           │   └── calibration.h         ← calibration servos
│           ├── os/                       ← OS kernel (cycles, events, jobs, commands)
│           ├── services/
│           │   ├── jetson/
│           │   │   ├── jetson_bridge.h   ← service Jetson (dispatch, télémétrie, watchdog)
│           │   │   └── jetson_bridge.cpp
│           │   ├── motion/               ← steppers, PID, kinematics, stallDetector
│           │   ├── safety/               ← détection obstacles
│           │   ├── lidar/                ← occupancy map (via T4.0)
│           │   ├── intercom/             ← protocole UART (T4.1 ↔ T4.0)
│           │   ├── actuators/            ← servos, pompes
│           │   ├── chrono/               ← timer match (100s)
│           │   └── …
│           ├── program/
│           │   ├── routines.cpp          ← boot/manual/auto/stopped routines
│           │   ├── strategy.cpp          ← stratégie embarquée (fallback C++)
│           │   ├── block_registry.h/.cpp ← registre blocs C++ (query par Jetson)
│           │   ├── mission.h/.cpp        ← Mission planner C++
│           │   └── mission_controller.h/.cpp ← exécuteur SD fallback
│           └── utils/
│               └── interpreter/          ← parseur de commandes texte
│
└── software/                             ← Python Jetson + Simulateur
    ├── shared/
    │   ├── config.py             ← constantes (miroir settings.h + poi.h)
    │   └── protocol.py           ← encode/decode trames CRC8 (compact + legacy)
    ├── transport/
    │   ├── base.py               ← interface abstraite Transport
    │   ├── xbee.py               ← transport XBee série (heartbeat tolérant 3 échecs)
    │   ├── wired.py              ← transport USB-CDC (alias XBeeTransport)
    │   └── virtual.py            ← transport virtuel (simulateur)
    ├── services/
    │   ├── motion.py             ← MotionService (API = C++)
    │   ├── safety.py             ← SafetyService
    │   ├── vision.py             ← VisionService
    │   ├── chrono.py             ← ChronoService
    │   └── occupancy.py          ← OccupancyService + carte de l'adversaire
    ├── sim/
    │   ├── physics.py            ← physique robot holonome (60Hz)
    │   ├── world.py              ← OccupancyGrid, Pathfinder A*, GameObjects
    │   ├── bridge.py             ← SimBridge (fake Teensy pour le sim)
    │   └── static/               ← UI web (HTML/JS/CSS)
    ├── strategy/
    │   ├── missions.json         ← définitions de missions (pour Mission planner)
    │   ├── macros.json           ← séquences de macro steps
    │   └── match.py              ← ★ Stratégie HOT-RELOADABLE
    ├── brain.py                  ← Orchestrateur Jetson
    ├── run.py                    ← Point d'entrée unifié (PC + Jetson)
    └── requirements.txt
```

---

## Modes de fonctionnement

### Dual Strategy : Remote (Jetson) vs Internal (C++)

Le switch IHM `strategySwitch` sélectionne le mode :

- **strat=1 (Remote / Intelligent)** : Le Jetson exécute `strategy/match.py`, envoie les commandes motion/actuators au Teensy. Le fallback watchdog est actif.
- **strat=0 (Internal / Séquentiel)** : Le Teensy exécute les blocs C++ enregistrés dans `BlockRegistry` via `match()`. Le fallback watchdog est désactivé. Le Jetson reste connecté pour la télémétrie et la webapp.

La webapp détecte le mode via la commande `health` (champ `strat=0|1`) après `match_start`.

### Mode Remote Controlled (strat=1)

```
Jetson connecté (heartbeat ok, strat=1)
        ↓
programAuto() sur Teensy attend les commandes
        ↓
Jetson exécute strategy/match.py
  motion.go(x,y) → Serial → Teensy → motion.go() → wait done → T:m DONE:ok
        ↓
Teensy répond une fois le mouvement terminé (blocking côté Jetson)
```

### Mode Internal (strat=0)

```
match_start via webapp ou tirette
        ↓
programAuto() détecte strat=0
        ↓
BlockRegistry.buildMission() → Mission planner C++
  → exécute les blocs par priorité/score
  → T:m visible sur la webapp (position + target en temps réel)
```

### Mode Fallback (Jetson déconnecté, strat=1)

```
Jetson timeout (> 5s sans heartbeat, mode Remote)
        ↓
JetsonBridge._checkWatchdog() fires (seulement si os.state==AUTO && strat=1)
        ↓
triggerFallback(FallbackID::CUSTOM_1)
  → si SD strategy chargée : MissionController::execute()
  → sinon : motion.cancel() + disengage
```

Les fallbacks sont enregistrables dans `onRobotBoot()` :
```cpp
jetsonBridge.registerFallback(FallbackID::RETURN_TO_BASE, []() {
    motion.cancel();
    async motion.go(POI::home);
});
```

### Mode Simulateur (Windows / Linux dev)

```
run.py --sim
    ↓
VirtualTransport ←→ SimBridge (fake Teensy)
    ↓
Brain (strategy/match.py) via les mêmes services
    ↓
Flask + SocketIO → http://localhost:5000
```

---

## Démarrage rapide

### Simulateur (Windows)

```bash
cd software/
pip install -r requirements.txt
python run.py --sim
# Ouvrir http://localhost:5000
```

- Clic gauche sur la table → déplace le robot
- Clic droit → place un obstacle
- Bouton "Run" → lance strategy/match.py
- Modifier `strategy/match.py` et sauvegarder → hot-reload automatique

### Jetson (hardware réel)

```bash
cd software/
pip install -r requirements.txt
python run.py                             # auto-détecte Linux → connect /dev/ttyUSB0
python run.py --connect /dev/ttyTHS1      # port spécifique
python run.py --auto-start                # auto-connect + start match
python run.py --no-auto-connect           # sur Jetson, ne pas auto-connecter
```

### PC avec robot en USB

```bash
python run.py --connect COM6
python run.py --connect COM6 --baud 57600  # baud rate spécifique
```

Ports courants sur Jetson :
- USB dongle XBee : `/dev/ttyUSB0`
- Header UART Jetson Nano : `/dev/ttyTHS1`
- Header UART Jetson Orin : `/dev/ttyTHS0`

---

## Écrire la stratégie (`strategy/match.py`)

La stratégie s'écrit de façon **identique** pour le simulateur et le vrai robot :

```python
def block_collect_A() -> BlockResult:
    # Vérifier la couleur avant de bouger
    color = vision.query_color_sync(POI.stock_1, timeout_ms=400)
    if not is_useful_color(color):
        return BlockResult.FAILED

    # Vérifier que la zone est libre (occupancy map du Jetson)
    if occupancy.is_zone_occupied(POI.stock_1, radius=450):
        return BlockResult.FAILED

    # Déplacement — blocking jusqu'à arrivée ou collision
    motion.cancel_on_collide().feedrate(0.7).go(POI.stock_1.x, POI.stock_1.y + 300)
    if not motion.was_successful():
        return BlockResult.FAILED

    return BlockResult.SUCCESS

def run_mission():
    safety.enable()
    m = Mission(chrono, log)
    m.set_mode(Mission.PRIORITY)
    m.add(Block("collect_A", priority=10, score=80, time_ms=7000,
                action=block_collect_A,
                feasible=lambda: not occupancy.is_zone_occupied(POI.stock_1, 450)))
    m.run()
```

---

## RuntimeConfig (SD card)

Fichier `/config.cfg` sur la SD card, format clé=valeur :

```
# Telemetry rates (ms)
tel.pos_ms=100
tel.motion_ms=100
tel.safety_ms=500
tel.chrono_ms=1000
tel.occ_ms=500

# Servo limits
servo.CA.0.min=110
servo.CA.0.max=170

# Motion
motion.max_speed=2800
```

Commandes terminal : `cfg_list`, `cfg_set(key,value)`, `cfg_save`.
Chargé automatiquement au boot après SDCard::init().

---

## BlockRegistry (C++) — Blocs exécutables

Registre statique de blocs C++ accessibles depuis le Jetson :

```cpp
// Dans onRobotBoot() :
BlockRegistry::instance().add("collect_A", 10, 80, 7000,
    []() -> BlockResult {
        async motion.go(1000, 500);
        // ... actions ...
        return BlockResult::SUCCESS;
    },
    []() -> bool { return !occupancy.isZoneOccupied(1000, 500, 450); }
);
```

Commandes bridge : `blocks_list`, `run_block(collect_A)`, `block_done(collect_A)`.
La stratégie embarquée `match()` utilise `BlockRegistry::buildMission()` pour construire un `Mission` à partir des blocs non-done.

---

## StallDetector — Détection de blocage

Détecte si le robot est bloqué en mesurant le déplacement sur une fenêtre glissante.

Seuils dans `Settings::Motion::Stall` :
```cpp
DELAY_MS         = 1000    // grace period avant première vérification
PERIOD_MS        = 500     // intervalle entre checks
TRANS_DISP_MM    = 5.0     // déplacement minimum attendu par fenêtre (mm)
ANGLE_DISP_RAD   = 0.02    // rotation minimum attendue par fenêtre (rad)
TARGET_TRANS_MM  = 20.0    // ignore si la cible est < 20mm
TARGET_ANGLE_RAD = 0.05    // ignore si la rotation cible est < 0.05 rad
```

Contrôle par move :
```cpp
async motion.noStall().go(x, y);       // désactive pour ce move
async motion.withStall(false).go(x,y); // idem
```

Désactivation globale : mettre `stallEnabled = false` dans `MoveOptions` (motion.h).

Calibration : `motion.printDiagReport()` affiche `stallMinTransMm` — mettre `TRANS_DISP_MM` légèrement en-dessous.

---

## JetsonBridge (C++) — Référence

### Enregistrer un fallback personnalisé

```cpp
// Dans onRobotBoot(), après os.attachService(&jetsonBridge) :
jetsonBridge.registerFallback(FallbackID::CUSTOM_1, []() {
    motion.cancel();
    safety.enable();
    motion.feedrate(0.5f);
    async motion.go(POI::home);
});
```

### Déclencher un fallback depuis le Jetson

```python
# Dans strategy/match.py côté Jetson :
transport.fire("fb(2)")  # déclenche FallbackID::RETURN_TO_BASE
```

### Vérifier l'état de connexion (Teensy)

```cpp
if (jetsonBridge.isRemoteControlled()) {
    // Jetson connecté, heartbeat reçu < 5s
}
```

---

## FAQ

**Q: Le XBee 868 MHz ne risque pas d'être saturé en compétition ?**

Les modules XBee 868/900 MHz utilisent une bande séparée du WiFi 2.4 GHz, ce qui évite les interférences en salle de compétition.

**Q: Que se passe-t-il si la connexion coupe en plein match ?**

1. Le Teensy détecte l'absence de heartbeat (> 5s) via le watchdog de JetsonBridge.
2. Il déclenche automatiquement le fallback enregistré (par défaut : arrêt moteurs).
3. Si `FallbackID::CUSTOM_1` est configuré avec une mission SD, le robot exécute la stratégie embarquée.
4. Le fallback ne se déclenche QUE en mode Remote (strat=1). En mode Internal (strat=0), le C++ gère déjà le match.
5. Si la connexion revient, le Jetson peut reprendre le contrôle.

**Q: Comment ajouter un nouveau service sur le Teensy ?**

1. Créer `firmware/teensy41/src/services/monservice/monservice.h/.cpp`.
2. Ajouter `ID_MONSERVICE` dans `service.h` (avant `ID_NOT_A_SERVICE`).
3. Ajouter le cas dans `service.cpp` (`toID` / `toString`).
4. Inclure dans `config/env.h`.
5. Attacher dans `onRobotBoot()` : `os.attachService(&monservice)`.

**Q: Comment tester uniquement la stratégie sans le robot ?**

```bash
python run.py --sim
# Puis dans l'UI : bouton "Run"
```

Le simulateur physique tourne à 60Hz avec détection de collisions, A* pathfinding, safety lookahead, et chrono. `strategy/match.py` est hot-reloadé à chaque sauvegarde.

**Q: Comment configurer les fréquences de télémétrie ?**

Ajouter les clés dans `/config.cfg` sur la SD card du Teensy :
```
tel.pos_ms=200
tel.safety_ms=1000
```
Les valeurs sont chargées au boot. Modifiables aussi à chaud via `cfg_set(tel.pos_ms,200)` + `cfg_save`.
