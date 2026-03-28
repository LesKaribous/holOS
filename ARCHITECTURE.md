# holOS — Architecture Remote-Controlled (2026)

## Vue d'ensemble

Le robot est piloté par deux cerveaux qui collaborent via XBee :

```
┌─────────────────────────────────────────────────────────────────┐
│                        JETSON  (py/)                            │
│                                                                 │
│  strategy/match.py   →  MotionService, SafetyService, …        │
│  Brain (brain.py)        (services/ — API identique sim/réel)  │
│  Transport abstrait  →  XBeeTransport (réel)                   │
│                          VirtualTransport (sim Windows)         │
└─────────────────────────────┬───────────────────────────────────┘
                              │  XBee  (Serial1, 31250 bps)
                              │  protocole CRC8-SMBUS
                              │
┌─────────────────────────────▼───────────────────────────────────┐
│                      TEENSY 4.1  (src/)                         │
│                                                                 │
│  Cyclic 10µs   →  motion.step()   (stepper ISR)                │
│  Cyclic 1ms    →  motion.control() (PID)                        │
│  OS loop       →  tous les services                             │
│                                                                 │
│  JetsonBridge  →  dispatch commandes, télémétrie, watchdog     │
│  Safety        →  lidar + occupancy (local, jamais délégué)    │
│  Fallback      →  si Jetson déconnecté → stratégie embarquée   │
└─────────────────────────────────────────────────────────────────┘
                              │  UART rapide
┌─────────────────────────────▼───────────────────────────────────┐
│                      TEENSY 4.0  (lidar)                        │
│  Génère l'occupancy map → envoie au Teensy 4.1                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## Protocole de communication XBee

Format des trames (identique au protocole Intercom existant) :

| Direction         | Format                           | Exemple                        |
|-------------------|----------------------------------|--------------------------------|
| Jetson → Teensy   | `<id>:<cmd>\|<crc8>\n`           | `42:go(500,300)\|123\n`         |
| Teensy → Jetson   | `r<id>:<response>\|<crc8>\n`     | `r42:ok\|45\n`                  |
| Télémétrie push   | `TEL:<type>:<data>\|<crc8>\n`    | `TEL:pos:x=500,y=300...\n`      |
| Heartbeat         | `ping\n` / `pong\n`              |                                |

CRC : **CRC8-SMBUS** (polynomial 0x07, init 0x00) — implémenté côté Python avec `crcmod`.

### Types de télémétrie (Teensy → Jetson)

| Type      | Données                              | Fréquence |
|-----------|--------------------------------------|-----------|
| `pos`     | `x=<mm>,y=<mm>,theta=<rad>`          | 10 Hz     |
| `motion`  | `IDLE` / `RUNNING` / `DONE:ok` / `DONE:fail` | event |
| `safety`  | `0` ou `1`                           | 10 Hz     |
| `chrono`  | `<elapsed_ms>`                       | 10 Hz     |
| `occ`     | hex bytes (occupancy map compressée) | 2 Hz      |

### Commandes supportées (Jetson → Teensy)

| Commande                         | Description                              | Réponse        |
|----------------------------------|------------------------------------------|----------------|
| `hb`                             | Heartbeat                                | `ok` immédiat  |
| `tel`                            | Snapshot télémétrie                      | `x=...,y=...`  |
| `occ`                            | Demande carte occupancy                  | hex bytes       |
| `go(<x>,<y>)`                    | Déplacement absolu                       | après fin move |
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
| `start`                          | Démarrer le match                        | `ok` immédiat  |
| `stop`                           | Arrêt robot                              | `ok` immédiat  |
| `fb(<id>)`                       | Déclencher un fallback (0-4)             | `ok` immédiat  |

---

## Structure des dossiers

```
holOS/
├── src/                          ← C++ Teensy 4.1 (PlatformIO)
│   ├── config/
│   │   ├── env.h                 ← includes globaux + JetsonBridge
│   │   ├── settings.h            ← constantes (baudrate, grille, PID…)
│   │   ├── pin.h                 ← mapping pins
│   │   └── poi.h                 ← points d'intérêt
│   ├── os/                       ← OS kernel (cycles, events, jobs)
│   ├── services/
│   │   ├── jetson/
│   │   │   ├── jetson_bridge.h   ← ★ NOUVEAU — service Jetson
│   │   │   └── jetson_bridge.cpp
│   │   ├── motion/               ← steppers, PID, kinematics
│   │   ├── safety/               ← détection obstacles
│   │   ├── lidar/                ← occupancy map
│   │   ├── intercom/             ← protocole XBee (existant)
│   │   ├── actuators/            ← servos, pompes
│   │   ├── chrono/               ← timer match
│   │   └── …
│   ├── program/
│   │   ├── routines.cpp          ← ★ REFACTORÉ — mode headless
│   │   ├── strategy.cpp          ← stratégie embarquée (fallback)
│   │   └── mission.cpp           ← Mission planner C++
│   └── utils/
│       └── interpreter/          ← parseur de commandes texte
│
└── py/                           ← ★ NOUVEAU — Python Jetson + Simulateur
    ├── shared/
    │   ├── config.py             ← constantes (miroir settings.h + poi.h)
    │   └── protocol.py           ← encode/decode trames CRC8
    ├── transport/
    │   ├── base.py               ← interface abstraite Transport
    │   ├── xbee.py               ← transport XBee série (Jetson réel)
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
    │   ├── mission.py            ← Mission planner (miroir C++)
    │   └── match.py              ← ★ Stratégie HOT-RELOADABLE
    ├── brain.py                  ← Orchestrateur Jetson
    ├── run_sim.py                ← Lancer le simulateur
    ├── run_jetson.py             ← Lancer sur le vrai Jetson
    └── requirements.txt
```

---

## Modes de fonctionnement

### Mode Remote Controlled (normal)

```
Jetson connecté (heartbeat ok)
        ↓
programAuto() sur Teensy attend les commandes
        ↓
Jetson exécute strategy/match.py
  motion.go(x,y) → XBee → Teensy → motion.go() → wait done → TEL:motion:DONE:ok
        ↓
Teensy répond uniquement une fois le mouvement terminé (blocking côté Jetson)
```

### Mode Fallback (Jetson déconnecté)

```
Jetson timeout (> 2s sans heartbeat)
        ↓
JetsonBridge.triggerFallback(FallbackID::STOP)
  → motion.cancel() + motion.disengage()
        ↓
programAuto() détecte !jetsonBridge.isRemoteControlled()
        ↓
match() embarqué s'exécute (strategy.cpp sur Teensy)
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
run_sim.py
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
cd py/
pip install -r requirements.txt
python run_sim.py
# Ouvrir http://localhost:5000
```

- Clic gauche sur la table → déplace le robot
- Clic droit → place un obstacle
- Bouton "Run" → lance strategy/match.py
- Modifier `strategy/match.py` et sauvegarder → hot-reload automatique

### Jetson (hardware réel)

```bash
cd py/
pip install -r requirements.txt
python run_jetson.py --port /dev/ttyUSB0
# ou
python run_jetson.py --port /dev/ttyTHS1 --auto-start
```

Ports XBee courants sur Jetson :
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
    // Jetson connecté, heartbeat reçu < 2s
}
```

---

## FAQ

**Q: Le XBee 2.4GHz ne risque pas d'être saturé en compétition ?**

Les XBee utilisent DSSS (Direct Sequence Spread Spectrum) qui donne une immunité aux interférences bien supérieure au WiFi brut. Pour encore plus de robustesse, utiliser les modules XBee-PRO 900HP ou SX (868/900 MHz) : bande totalement libre en salle de compétition robotique.

**Q: Que se passe-t-il si la connexion coupe en plein match ?**

1. Le Teensy détecte l'absence de heartbeat (> 2s) via le watchdog de JetsonBridge.
2. Il déclenche automatiquement le fallback enregistré (par défaut : arrêt moteurs).
3. Si `FallbackID::RETURN_TO_BASE` est configuré, le robot rentre à sa zone de départ.
4. Si la connexion revient, le Jetson peut reprendre le contrôle dès le prochain heartbeat.

**Q: Comment ajouter un nouveau service sur le Teensy ?**

1. Créer `src/services/monservice/monservice.h/.cpp`.
2. Ajouter `ID_MONSERVICE` dans `service.h` (avant `ID_NOT_A_SERVICE`).
3. Ajouter le cas dans `service.cpp` (`toID` / `toString`).
4. Inclure dans `config/env.h`.
5. Attacher dans `onRobotBoot()` : `os.attachService(&monservice)`.

**Q: Comment tester uniquement la stratégie sans le robot ?**

```bash
python run_sim.py
# Puis dans l'UI : bouton "Run"
```

Le simulateur physique tourne à 60Hz avec détection de collisions, A* pathfinding, safety lookahead, et chrono. `strategy/match.py` est hot-reloadé à chaque sauvegarde.
