# firmware.md — holOS Robot Firmware Knowledge Base

> Base de connaissances générée par l'assistant pour naviguer le firmware du robot holonome (Coupe de France de Robotique).
> Dernière mise à jour : 2026-04-18

Ce document est le point d'entrée pour comprendre le firmware. Il consolide l'architecture, les conventions, les paramètres clés, les protocoles de communication, et les points d'attention par zone de code. Il est complémentaire à `FIRMWARE_DOCUMENTATION.md` (déjà présent, plus descriptif).

---

## 1. Vue d'ensemble

Le projet "holOS" pilote un robot holonomique à 3 moteurs pas-à-pas (120° entre eux) pour la Coupe de France de Robotique. L'architecture est bi-Teensy :

```
                   XBee 868MHz              UART Intercom (Serial1/Serial2)
  Jetson / PC  <------------------->  T4.1  <-----------------------------> T4.0
  (holOS)         ou USB-CDC          Contrôleur principal                  Perception
                                      - Moteurs (3 steppers holo)           - LIDAR LD06
                                      - Stratégie / Mission                 - Carte d'occupation
                                      - Actionneurs (servos, pompes)        - NeoPixel (anneau 35 LEDs)
                                      - Safety / Chrono                     - OLED SH1106
                                      - JetsonBridge                        - Intercom
```

Les deux cartes partagent la même philosophie : un petit OS coopératif avec des **Services** singletons, une machine à états de quelques routines, et un modèle de **Job** asynchrone. Le T4.1 est beaucoup plus riche (contrôle temps réel, cinématique, planification de missions).

Toolchain : **PlatformIO + Arduino framework (Teensyduino)**, C++ avec `std::` (map, vector, queue, stack). Compilation optimisée en taille (`-Os`) avec garbage collection à la link pour le T4.1.

---

## 2. Organisation du dépôt

```
firmware/
├── FIRMWARE_DOCUMENTATION.md   ← doc officielle (descriptive, à jour)
├── firmware.md                 ← ce fichier (knowledge base)
├── teensy40/
│   ├── platformio.ini
│   ├── note.md                 ← schéma ASCII du châssis et repères
│   ├── docs/  pcb/  sim/
│   └── src/
│       ├── main.cpp            ← setup/loop + onBoot + handlers intercom
│       ├── pin.h   settings.h  ← config plate
│       ├── os/                 ← OS, console, commandes, job queue
│       ├── services/
│       │   ├── lidar/          ← LD06 driver, filtres, grilles, occupancy
│       │   ├── pixel/          ← NeoPixel (modes INTERCOM/LIDAR/TEAM)
│       │   ├── intercom/       ← lien UART vers T4.1
│       │   └── terminal/
│       └── utils/              ← geometry, interpreter, timer, job
└── teensy41/
    ├── platformio.ini          ← 3 envs : teensy35_old_board, teensy35, teensy41 (default)
    └── src/
        ├── main.cpp            ← setup + cycle_manager start + os.run()
        ├── config/             ← settings, pin, env, poi, score, text,
        │                          calibration, runtime_config
        ├── os/                 ← OS + singleton + cycles + threads + jobs + debug
        ├── program/            ← block_registry, mission, routines, strategy
        ├── services/           ← 13 services (voir §5)
        └── utils/              ← commandHandler, geometry/, interpreter/, planner/, timer/
```

Points d'attention :
- Deux boards historiques sont encore supportées par le `platformio.ini` du T4.1 (`teensy35`, `teensy35_old_board`), mais l'environnement par défaut est `teensy41`. Les vieilles configs utilisent `ILI9341_t3n.git` (KurtE) au lieu de `ILI9341_t3.git` (PaulStoffregen) pour le T4.1.
- Le flag de compilation `OLD_BOARD` a une influence sur le pin mapping dans `config/pin.h`.
- Le T4.0 a un `platformio.ini` minimaliste (un seul env `teensy4`).

---

## 3. Teensy 4.1 — Contrôleur Principal

### 3.1 Boot et machine à états

Fichier d'entrée : `teensy41/src/main.cpp`.

```
setup()
  Console::init() au niveau VERBOSE
  enregistrement des routines (BOOT, MANUAL, MANUAL_PROGRAM, AUTO_PROGRAM, AUTO, STOPPED)
  cycle_manager.registerCycle(T_10US, step)     // pulse des steppers
  cycle_manager.registerCycle(T_1MS, control)   // boucle PID
  cycle_manager.start()                         // threads TeensyThreads

loop()
  os.run()                                      // dispatche vers la routine de l'état courant
```

Les états (enum dans `os/os.h`) :

| État | Rôle |
|------|------|
| `BOOT` | Une seule passe, appelle `onRobotBoot()` (attach de tous les services) |
| `MANUAL_PROGRAM` | Programme de préparation (choix équipe, stratégie, recalage) |
| `MANUAL` | Idle pré-match (attend le retrait de la tirette) |
| `AUTO_PROGRAM` | Démarrage du match : initialisations |
| `AUTO` | Match en cours : stratégie locale ou télé-opérée |
| `STOPPED` | Fin de match (100 s écoulées) |

Transitions clés :
- `BOOT → MANUAL_PROGRAM` : à la fin de `onRobotBoot()`.
- `MANUAL_PROGRAM → MANUAL` : après le programme de préparation.
- `MANUAL → AUTO_PROGRAM` : tirette retirée (starter pin).
- `AUTO → STOPPED` : Chrono déclenche `onMatchEnd()` à 100 s.
- Forçage possible via les commandes `start` / `stop` du terminal.

### 3.2 CycleManager et threads

`os/cycles.{h,cpp}` définit 8 bandes de fréquence (`T_1US`, `T_10US`, `T_100US`, `T_1MS`, `T_10MS`, `T_100MS`, `T_1S`, `T_10S`). Chaque cycle enregistré est exécuté dans un thread dédié (TeensyThreads) avec une stack de 8 Ko ; la tranche de temps du scheduler est de 10 µs (`threads.setSliceMicros(10)`).

Utilisations critiques :
- `T_10US → step()` : génération ISR-level des impulsions des 3 steppers (100 kHz).
- `T_1MS → control()` : boucle PID / velocity (`PID_INTERVAL = 2000 µs` dans les settings, mais le cycle tourne à 1 ms, ce qui est un point à vérifier côté cohérence).
- Boucle principale `os.run()` ~2 ms.

### 3.3 OS et Services

`os/os.{h,cpp}` est un singleton qui gère :
- `std::map<ServiceID, Service*>` des services attachés,
- `std::stack<Job*>` des jobs (LIFO) — tout mouvement ou script est un Job,
- les routines des états (function pointers),
- un interpréteur (`Interpreter`) pour exécuter des scripts reçus.

API principale :
- `attachService(Service*)` : enregistrement + `attach()` + `enable()`.
- `updateServices()` : appel `run()` de chaque service non-threaded.
- `wait(Job&, bool runasync)` : empile un job ; bloque jusqu'à complétion si `runasync == false`.
- `execute(String& rawcmd)` : interprète une ligne de script.
- `isBusy()` : `true` si au moins un job en attente/en cours.

Cycle de vie d'un Job : `IDLE → PENDING → RUNNING → (PAUSING → PAUSED) → COMPLETED | CANCELED`.

### 3.4 Services enregistrés (13)

Définis dans `services/service.{h,cpp}` (classe de base avec `attach()` / `run()` / `enable()` / `disable()` / flags `m_enabled`, `m_threaded`, `m_debug`).

| ID (`ServiceID`) | Classe | Rôle principal | Dépendances clés |
|------------------|--------|----------------|------------------|
| `ID_MOTION` | `Motion` | Cinématique holo, PID, stepper control, options de mouvement | Localisation, StallDetector |
| `ID_SAFETY` | `Safety` | Déclenche l'arrêt si obstacle détecté | Lidar (carte d'occupation) |
| `ID_CHRONO` | `Chrono` | Minuteur de match (100 s) avec callbacks | — |
| `ID_ACTUATORS` | `Actuators` | Servos / pompes / EV via PCA9685 | I2C |
| `ID_LIDAR` | `Lidar` | Côté T4.1 : relais + requête de la carte d'occupation | Intercom |
| `ID_VISION` | `Vision` (stub) | Interrogation couleur déléguée à la Jetson | JetsonBridge |
| `ID_IHM` | `IHM` | TFT ILI9341 + boutons + switches + buzzer | — |
| `ID_INTERCOM` | `Intercom` | UART T4.1 ↔ T4.0 (31250 baud) | Serial1 |
| `ID_JETSON` | `JetsonBridge` | Commande distante + télémétrie + fallback | USB / XBee |
| `ID_TERMINAL` | `Terminal` | Console série / commandes | CommandHandler |
| `ID_LOCALISATION` | `Localisation` | Odométrie optique Qwiic OTOS | I2C |
| `ID_NEOPIXEL` | `Pixel` | Indicateur LED | — |
| `ID_NAVIGATION` | (réservé) | Non instancié dans ce build | — |

### 3.5 Système Moteur / Motion

`services/motion/motion.{h,cpp}` avec contrôleurs dans `services/motion/controller/` :

- `PositionController` (mode croisière) : trois PID indépendants
  - `vx` : P=4.0, I=0.0, D=100.0
  - `vy` : P=4.0, I=0.0, D=100.0
  - `vrot` : P=10.0, I=0.0, D=70.0

  Conversion des erreurs `(x, y, θ)` → vitesses cartésiennes → `ik()` → vitesses roues (a, b, c), puis `VelocityController` ramène ça vers les pulses des steppers.

- `StepperController` (fallback open-loop) : profils trapézoïdaux via `TwinsyStep` (fork de TeensyStep). Utilisé si l'OTOS n'est pas disponible.

- `PursuitController` (mode LIVE_PURSUIT) : le point cible est réactualisé en continu par la Jetson via `aim(x, y)` ; pas de décélération automatique (c'est l'hôte qui gère l'arrêt). Watchdog 200 ms : si pas d'`aim()` reçu, vitesse = 0.

- `VelocityController` : mapping cartésien → holonomique.

Modes globaux : absolute/relative, sync/async, cruise/stepper-only, `setFeedrate(0.05..1.0)`, `enableAPF(scale)` (Artificial Potential Field : gradient répulsif depuis la carte d'occupation, ajouté à la consigne de vitesse).

API fluent pratique :
```cpp
async motion.noStall().feedrate(0.8f).via(200, 0).via(500, 300).go(500, 300);
async motion.withStall().cancelOnStall().go(x, y);
async motion.goPolar(heading, distance);
async motion.goAlign(target, RobotCompass::A, 0);   // aligne la face A sur la cible
```

Paramètres moteur (de `config/settings.h`, valeurs effectives côté code) :

| Paramètre | Valeur |
|-----------|--------|
| `MAX_SPEED` | 3800 mm/s (plafonné par OTOS à 2500 mm/s) |
| `MAX_ACCEL` | 3500 mm/s² |
| `MAX_ROT_SPEED` | 10 rad/s |
| `MAX_ROT_ACCEL` | 30 rad/s² |
| `MIN_DISTANCE` (tolérance XY) | 20 mm |
| `MIN_ANGLE` (tolérance θ) | 2° (≈ 0.035 rad) |
| `PID_INTERVAL` | 2000 µs |
| Steps/rev | 200 (NEMA17) |
| Microstepping | 8× |
| `MIN_STEP_DELAY` | 20 µs |
| `PULSE_WIDTH` | 14 ns (dans settings ; en pratique 14 µs — à valider) |
| `MAX_ACCEL` stepper | 3000 steps/s² |
| `MAX_SPEED` stepper | 15000 steps/s |
| Géométrie | `RADIUS = 125.98 mm`, `WHEEL_RADIUS = 30 mm` |

> NB : la valeur `PULSE_WIDTH = 14 ns` dans `settings.h` est manifestement une confusion d'unité (la doc officielle parle de 14 µs). À revérifier à l'usage.

`MoveStats` est enregistré à chaque mouvement terminé (`durationMs`, distances cibles vs parcourues, `stalled`, compteurs de saturation PID par axe). `motion.printDiagReport()` produit le rapport formaté.

**Stall detection** (`services/motion/stallDetector.{h,cpp}`) : trois méthodes complémentaires.
1. Fenêtre glissante legacy : déplacement < `TRANS_DISP_MM` (5 mm) ou < `ANGLE_DISP_RAD` (0.02 rad) par fenêtre de 500 ms, après délai initial de 1000 ms.
2. Velocity mismatch : vitesse commandée (sortie PID) > 30 mm/s mais vitesse mesurée par l'OTOS < 10 mm/s pendant 200 ms. Permet de détecter les contacts de mur très rapidement.
3. Error stagnation : l'erreur PID ne décroît pas de plus de 0.5 mm par 100 ms, seuil de temps ~400 ms. Utile aux faibles vitesses où le velocity mismatch échoue.

**Border snap** : après contact mur, on peut "claquer" la position mesurée à une valeur connue sur un axe (`snapAxisTarget(axis, value)`) et clear le stall pour ne pas re-déclencher.

Timeout mouvement : 5000 ms, **actuellement hardcodé** dans `motion.cpp` (TODO signalé : passer par `RuntimeConfig`).

### 3.6 JetsonBridge (télécommande et télémétrie)

`services/jetson/jetson_bridge.{h,cpp}`.

- Auto-détection du transport : premier port (USB `Serial` ou XBee `Serial2`) qui reçoit un frame valide est retenu comme `g_bridgeSerial` (global pointer). Baudrate : **57600 par défaut**, configurable.
- Heartbeat : la Jetson doit envoyer `hb` régulièrement. Timeout **5000 ms** (pas 2000, malgré ce qu'indique la doc historique) → `FallbackID` déclenché.
- Fallbacks enregistrables : `STOP`, `WAIT_IN_PLACE`, `RETURN_TO_BASE`, `CUSTOM_1`, `CUSTOM_2`. Le flag `isRemoteControlled()` gate l'exécution de `match()` : si la Jetson est présente, la stratégie locale ne s'exécute pas.
- Télémétrie : canaux individuels activables (`pos`, `motion`, `safety`, `chrono`, `occ`), agrégation des 4 premiers à **10 Hz (100 ms)**, carte d'occupation à **2 Hz (500 ms)**. Frames CRC-SMBUS : `TEL:channel:data|crc\n`.
- Complétion de mouvement : `TEL:motion:DONE:ok|crc` ou `DONE:fail|crc` ; retransmis toutes les 2 s jusqu'à ACK côté hôte (robustesse XBee).
- Ring buffer de 16 × 256 octets pour les frames sortantes → pas d'alloc dynamique en runtime, et pas de réentrance pendant le parsing.

### 3.7 Intercom T4.1 ↔ T4.0

`services/intercom/intercom.{h,cpp}` + `request.{h,cpp}` + `comUtilities.{h,cpp}`.
- Transport : `INTERCOM_SERIAL = Serial1` à **31250 baud** (vitesse MIDI, pratique côté T4.0).
- Framing : `uid:content|crc8\n` pour les requêtes, `ruid:content|crc8\n` pour les réponses, `content\n` pour les messages plain. CRC : SMBUS (FastCRC).
- Handshake : `ping\n` → `pong\n`. Connexion perdue après 2 s sans trafic.
- Pool de requêtes : `MAX_PENDING = 8`, zéro heap. Callback `requestCallback_ptr` + `callback_ptr`.
- Callbacks globaux : `setConnectLostCallback`, `setConnectionSuccessCallback`, `setRequestCallback`.

Requêtes émises par le T4.1 vers le T4.0 :

| Commande | Payload | Retour | Usage |
|----------|---------|--------|-------|
| `pos` | `pos(x,y,θ)` | `OK` | Mise à jour position robot (repère LIDAR) |
| `ob` | `ob(angle,distMax)` | bool | Test d'obstacle dans un cône |
| `gD` | `gD(angle)` | int | Distance brute à un angle |
| `oM` | `oM` | hex | Carte d'occupation complète compressée |
| `oD` | `oD` | `"gx,gy;..."` | Obstacles dynamiques en sparse |
| `sM` | `sM(hexBitmap)` | `OK` | Charge la carte statique (par équipe) |
| `team` | `team(Y|B)` | — | Définit la couleur d'équipe (affichage + statique) |
| `on` / `off` | — | `OK` | Switch du mode d'affichage NeoPixel (LIDAR / INTERCOM) |

### 3.8 Mission Planner et Stratégie

Le dossier `program/` contient la logique de match (au-dessus des services).

`block_registry.{h,cpp}` : base de "blocs" (max 16), chacun avec `name`, `priority`, `score`, `estimatedMs`, `action` (`BlockResult (*)()`), `feasible` (`bool (*)()`), flags `done` et `used`. Serialisable texte (pour exposition à la Jetson).

`mission.{h,cpp}` : une `Mission` est une séquence de `Step` (action + feasibility + flag `cancelable`). Un `Planner` agrège plusieurs missions :
- `setTimeProvider(uint32_t (*)())` (p.ex. `chrono.getTimeLeft`).
- `setSafetyMargin(uint32_t ms)` défaut **3000 ms** (empêche de démarrer une mission qui ne tient pas dans le temps restant).
- `setSafetyAbortMs(uint32_t ms)` défaut **8000 ms** (timeout si l'étape est bloquée par Safety).
- Sélection : priorité décroissante parmi les missions faisables. Si une étape échoue → retry après cooldown (**3000 ms**), puis `ABANDONED` après `maxRetries` (255 = infini).
- Étapes non cancelables (p.ex. robot en train de transporter un objet) ignorent le timeout de Safety.

`strategy.{h,cpp}` : point d'entrée `match()` (appelé si pas remote control), `recalage()`, `registerBlocks()`, `calibrate()`. Choisit les POIs via `choose(teamIsYellow, yellowPOI, bluePOI)`.

`routines.{h,cpp}` : points d'entrée des états (`onRobotBoot`, `onRobotManual`, `onRobotAuto`, `onRobotStop`, `programManual`, `programAuto`) et des callbacks d'événements (`onIntercomRequest`, `onMatchNearEnd`, `onMatchEnd`, `step`, `control`).

### 3.9 Autres services

- **IHM** : TFT ILI9341 (CS=10, DC=9, MOSI=11, SCK=13, MISO=12), switches (team, strategy, twin, starter), bouton reset, buzzer. Plusieurs pages : BOOT, INIT, MATCH, RESET. Mélodies (`playStartupMelody`, etc.).
- **Chrono** : hérite `Service` + `Timer`. `setNearEndCallback` (10 s avant la fin) + `setEndCallback`. Durée du match : **100 s**. Buffer de fin de match : 200 ms.
- **Localisation (OTOS)** : SparkFun Qwiic OTOS sur I2C. Calibration : échelle linéaire (par défaut 0.990723 dans la doc, 1.022 dans le code — valider le canonique) et angulaire. Expose `setPosition`, `getPosition`, `getVelocity`, `calibrate`, flag `useIMU()`. Vitesse max trackée 2.5 m/s.
- **Safety** : surveille la carte d'occupation / les switchs de collision ; expose `obstacleDetected()`.
- **Vision** : *stub* — toute la vision tourne sur la Jetson. Les appels `queryColorSync`, `requestAI` retournent `UNKNOWN`.
- **Actuators** : trois `ActuatorGroup` (AB / BC / CA), chacun avec des `SmartServo` commandés via PCA9685 (I2C PWM). Poses nommées (`DROP`, `GRAB`, `STORE`, `UP`, `DOWN`). Gestion pompe/EV encore en dehors du service (TODO à refactorer).
- **SD** : retiré du flux principal ; mission via buffer mémoire (`loadFromString`, 8 Ko max).
- **Terminal** : enregistrement dynamique via `CommandHandler` (map nom → func + description + syntaxe + args). Commandes : motion (`go`, `turn`, `feedrate`, `stall`), config (`cfg_set/get/list`, `calib_*`), telemetry (`tel`, `log`), diagnostic (`mission_*`, `block_*`, `help`).

### 3.10 Configuration et Pins (T4.1, `config/pin.h`)

| Pin | Fonction |
|-----|----------|
| 24 | Stepper enable |
| 25, 4, 5 | Stepper dir A, B, C |
| 27, 6, 26 | Stepper step A, B, C |
| 38 | Reset button |
| 32 | Starter (tirette) |
| 31 | Team switch |
| 36 | Strategy switch |
| 2 | Twin switch |
| 28 | Power Traco enable |
| 3 | Buzzer |
| 37 | NeoPixel data |
| 10/9/11/13/12 | TFT ILI9341 (CS/DC/MOSI/SCK/MISO) |

Servos PCA9685 : Servo_AB_1=16 (lift), Servo_AB_2=23 (gripper), Servo_CA_1=17 (elevator), Servo_CA_2=22 (right), Servo_CA_3=21 (left), Servo_BC_4=20 (thermomètre). Canaux PCA9685 0..3 pour `PUMP_CA_RIGHT`, `EV_CA_RIGHT`, `PUMP_CA_LEFT`, `EV_CA_LEFT`.

### 3.11 Points d'intérêt (POI) — thème 2025

Dans `config/poi.h` : spawns Yellow/Blue, variantes "ninja", cibles du thermomètre (source chaude et cible), pantry (10 positions), frigo (4 par équipe), zones de stockage. Outils utiles pour scripter des stratégies.

### 3.12 Libs et dépendances (T4.1)

`platformio.ini` (env `teensy41`) :
- `SPI`
- `SparkFun_Qwiic_OTOS_Arduino_Library` (localisation)
- `ILI9341_t3` (TFT)
- `FastCRC` (CRC SMBUS)
- `TwinsyStep` (fork TeensyStep, fourni par LesKaribous)
- `TeensyThreads` (multi-thread coopératif)
- `Adafruit-PWM-Servo-Driver-Library` (PCA9685)

Flags : `-D TEENSY41 -Os -ffunction-sections -fdata-sections -Wl,--gc-sections`, unflag `-O2`.

---

## 4. Teensy 4.0 — Perception

### 4.1 Boot et structure

`teensy40/src/main.cpp` :
```
setup()
  Console init (INFO)
  Routines : onBoot, onUpdate
onBoot()
  os.attachService(&lidar)
  os.attachService(&pixel)
  os.attachService(&intercom)
  intercom.setRequestCallback(onIntercomRequest)
loop()
  os.run()   // updateServices() + routine + job queue
```

OS plus simple qu'au T4.1 : deux états (`BOOT`, `RUNNING`), `std::map<ServiceID, Service*>`, `std::queue<Job*>` (FIFO). Service base class (`services/service.{h,cpp}`) avec hooks `onAttach()` et `onUpdate()`.

### 4.2 LIDAR LD06

`services/lidar/` :
- `ld06.{h,cpp}` : driver série, baud **230400**, paquet 47 octets (`0x54 0x2C ...`). 12 points par paquet (`PTS_PER_PACKET = 12`). Buffer circulaire de **512 points** max (`MAX_POINTS`). Persistance **500 ms** (`PERSISTENCE`).
  - Parsing : lecture octet par octet, validation CRC via table (`crc.h`, CrcTable 256 entrées, XOR avec LUT).
  - Interpolation linéaire des angles entre FSA et LSA pour obtenir l'angle de chacun des 12 points.
  - Transformation polaire → cartésien absolu avec la position du robot : `x = x_r + d·cos(a + θ)`, `y = y_r − d·sin(a + θ)`.
- `filter.{h,cpp}` :
  - `PolarFilter` : `minDist = 200 mm`, `maxDist = 1000 mm`, angles full-range, intensité min **100** (250 dans la doc descriptive — valider).
  - `CartesianFilter` : zone du terrain 3000×2000 mm, marge **70 mm**.
- `grid.{h,cpp}` :
  - `PolarGrid` : 36 secteurs de 10° ; chaque secteur garde jusqu'à 50 points, calcule une distance moyenne et un compte.
  - `CartesianGrid` : 20×13 cellules, décroissance de 1 par 100 ms, seuil d'occupation > **25**. Conversion `gx = (x·20)/3000`, `gy = (y·13)/2000`.
- `lidar.{h,cpp}` :
  - API : `getDistance(angle, absolute)`, `getCount(angle, absolute)`, `getOccupancyMap()` (bitmap 33 octets), `getOccupancyDyn()` (sparse), `setStaticMapHex(hexstr)` pour charger la carte d'équipe.
  - Affichage OLED (SH1106 128×64 via U8g2) mis à jour toutes les **500 ms**.
  - Reset des stats toutes les **1000 ms**.

### 4.3 Algorithme de détection d'obstacles (vote multi-rayons)

Handler de la requête `ob(angle, distMax)` dans `main.cpp` :
- 9 rayons testés : centre, ±10°, ±20°, ±30°, ±40°.
- Un rayon "vote obstacle" si distance < seuil **ET** count > 2.
- Seuils avec marge de décélération progressive : ±10° : `distMax` ; ±20° : `-20 mm` ; ±30° : `-35 mm` ; ±40° : `-55 mm`.
- OR logique entre tous les rayons : dès qu'un seul vote, obstacle détecté.

### 4.4 Carte d'occupation compressée

Grille 20×13 = 260 bits empaquetés en 33 octets (y-outer, x-inner, LSB first).
Format transmis : `"hexpayload-crc"` où CRC est un XOR simple des octets (code dans `comUtilities.cpp`), différent du SMBUS utilisé pour le framing Intercom. Combine la carte dynamique LIDAR avec la carte statique (chargée via `sM` / `team`).

### 4.5 NeoPixel (35 LEDs)

`services/pixel/` :
- Pin 4, WS2812B GRB, 800 kHz.
- Modes :
  - `INTERCOM` : vert pulsé si connecté, rouge clignotant sinon.
  - `LIDAR` : ~9.7° par LED, dégradé rouge (≤ 300 mm) → vert (≥ 1000 mm).
  - `TEAM COLOR` : jaune ou bleu pulsé selon équipe.
- Animation : brightness min 5 / max 50, cycle 1000 ms, refresh 20 ms.
- Transition automatique INTERCOM → LIDAR 5 s après connexion.

### 4.6 OLED SH1106

128×64 (le code mentionne aussi 96×64 pour la surface rendue — à valider).
Afficheur U8g2 (I2C SDA=18, SCL=19). Dessine la grille 20×13 et un marqueur robot.

### 4.7 Intercom côté T4.0

`services/intercom/` : miroir du T4.1, mais sur **Serial2** à 31250 baud.
- Ping périodique 500 ms, perte après 2 s.
- UID auto-incrémenté.
- CRC SMBUS via FastCRC pour le framing.
- Retry des requêtes non acquittées toutes les 5 ms.

### 4.8 Pins (T4.0, `src/pin.h`)

| Pin | Fonction |
|-----|----------|
| 4 | NeoPixel data (35 LEDs) |
| 6 | LIDAR LD06 speed / enable (PWM) |
| 7 | LIDAR RX (Serial1 @ 230400 baud) |
| 18 | OLED SDA (I2C) |
| 19 | OLED SCL (I2C) |
| Serial2 | Intercom (31250 baud) |

### 4.9 Libs et dépendances (T4.0)

`platformio.ini` (env `teensy4`) :
- `featherfly/SoftwareSerial` (rescue pour les ports série logiciels)
- `adafruit/Adafruit NeoPixel`
- `olikraus/U8g2` (OLED)
- Implicites (Teensy core) : `Serial`, `Wire`, `FastCRC`.

Flag : `-D TEENSY40`.

### 4.10 Repères mécaniques (extrait de `teensy40/note.md`)

```
BF (arrière → avant) : 170 mm
CF (centre → avant)  : 60 mm
CB (centre → arrière) : 110 mm
RC = LC              : 110 mm (+ 6 mm plexi)
RL                   : ~240 mm
```
Repère terrain : origine en haut à gauche, X vers la droite, Y vers le bas.

---

## 5. Protocole de communication

### 5.1 Framing commun

```
<payload>|<crc>\n
```
où `<crc>` = représentation décimale de `CRC8_SMBUS(payload_bytes)`.

| Type | Format | Sens |
|------|--------|------|
| Requête | `uid:command|crc\n` | émetteur → récepteur |
| Réponse | `ruid:response|crc\n` | récepteur → émetteur |
| Télémétrie | `TEL:channel:data|crc\n` | firmware → hôte |
| Message simple | `message\n` | les deux sens, sans CRC ni ACK |
| Heartbeat / handshake | `ping\n`, `pong\n`, `hb` | les deux sens |

### 5.2 Couches physiques

| Lien | Baud | Port | Entre |
|------|------|------|-------|
| USB-CDC | 57600 (réglable) | `Serial` | T4.1 ↔ PC |
| Intercom | 31250 | Serial1 (T4.1) / Serial2 (T4.0) | T4.1 ↔ T4.0 |
| XBee 868 MHz | 57600 (ou 31250 selon déploiement) | Serial2 (T4.1) | T4.1 ↔ Jetson |

### 5.3 Handshakes

- USB-CDC : l'hôte envoie `hb` framé CRC ; toute réponse valide confirme la connexion.
- XBee / Intercom : `ping` / `pong` toutes les 500 ms, timeout 2 s (Intercom) / 5 s (Jetson heartbeat).

---

## 6. Temps et cadences

| Constante | Valeur | Commentaire |
|-----------|--------|-------------|
| Durée du match | 100 s | |
| Near-end warning | 10 s avant la fin | |
| End-match buffer | 200 ms | Sécurité avant STOPPED |
| Mission safety margin | 3000 ms | `Planner::setSafetyMargin` |
| Mission safety abort | 8000 ms | `Planner::setSafetyAbortMs` |
| Retry cooldown | 3000 ms | Entre deux essais de Mission |
| Max retries | 255 (INFINITE) | Configurable par Mission |
| Stall delay initial | 1000 ms | Avant de commencer à tester |
| Stall window | 500 ms | Fenêtre glissante |
| Stall trans disp min | 5 mm / fenêtre | |
| Stall angle disp min | 0.02 rad / fenêtre | |
| Heartbeat Jetson timeout | 5000 ms | Déclenche `FallbackID` |
| Intercom connection lost | 2000 ms | Sans paquet reçu |
| LIDAR persistence | 500 ms (T4.0) / 1000 ms (T4.1 ref) | |
| NeoPixel refresh | 20 ms (50 Hz) | |
| Télémétrie aggregate | 100 ms (10 Hz) | |
| Télémétrie occupancy | 500 ms (2 Hz) | |

---

## 7. Logging (Console)

Niveaux : `VERBOSE(0) < INFO(1) < SUCCESS(2) < WARNING(3) < CRITICAL(4) < DISABLED(5)`.

Côté T4.1 :
- `Console::init()` au boot à `VERBOSE`, puis bascule au niveau `Settings::Log::BOOT_LEVEL = INFO`.
- Activation par source via bitmask `SRC_*` (LIDAR, CHRONO, IHM, SAFETY, MOTION, LOCALISATION, INTERCOM, NEOPIXEL).
- Par défaut bruyants (off) : `SRC_LOCALISATION`, `SRC_INTERCOM`, `SRC_NEOPIXEL`.
- Sur USB, `CONSOLE_SERIAL == BRIDGE_SERIAL` : les lignes non framées (`|crc`) sont redirigées vers holOS dans un canal dédié `_console`.
- Plotting Teleplot/Serial Studio : `Console::plot("name", value)` → `>name:value`, `Console::plotXY("n","x","y")` → `>name:x:y|xy`.

Côté T4.0 : `ConsoleStream` + `ConsoleLevel` (levels identiques), défaut `INFO`, sortie sur `Serial`.

---

## 8. TODO / FIXME repérés

| Fichier | Remarque |
|---------|----------|
| `teensy41/src/program/strategy.h:23` | "Integrate Pump and EV into Actuators" |
| `teensy41/src/program/strategy.cpp:10` | "déplacer dans Actuators" |
| `teensy41/src/program/strategy.cpp:322` | POI de debug à retirer |
| `teensy41/src/program/strategy.cpp:393` | "Integrate into Actuators" (initPump) |
| `teensy41/src/services/actuators/smartServo.cpp:52` | "safety exit based on max iteration" |
| `teensy41/src/services/motion/motion.cpp` | Timeout hardcodé à 5000 ms (RuntimeConfig commenté) |
| `teensy41/src/utils/geometry.cpp:53` | "TODO Optimize that shit" |
| `teensy40/src/utils/geometry/geometry2D.h:19` | Tentative de simplification des appels d'heading |

Points de cohérence à valider :
- `settings.h` indique `PULSE_WIDTH = 14 ns` (probable erreur d'unité — attendu 14 µs).
- Doc historique parle d'un timeout heartbeat de 2000 ms ; le code dit **5000 ms**.
- Intensité min du `PolarFilter` côté T4.0 : **100** dans le code vs **250** dans la doc descriptive.
- OLED : code U8g2 comparé à la résolution officielle 128×64 (le rendu utilise parfois 96×64).
- Échelle linéaire OTOS : 0.990723 (doc) vs 1.022 (code) — adopter une valeur canonique.

---

## 9. Références rapides

### 9.1 Ajouter une commande Terminal
1. Écrire `void command_xxx(const args_t& args)`.
2. `CommandHandler::registerCommand("xxx", "description", command_xxx);` (dans `terminal.cpp` ou dans la routine d'init).
3. Utilisable via Terminal série ET via JetsonBridge (le bridge fait transiter par `os.execute`).

### 9.2 Ajouter un Service T4.1
1. Classe héritant de `Service`, avec un `ServiceID` unique (étendre l'enum si besoin).
2. Macros `SINGLETON_EXTERN` dans le header, `SINGLETON_INSTANTIATE` dans le .cpp.
3. Dans `routines.cpp::onRobotBoot()`, `os.attachService(&monService)`.
4. Optionnel : flag `m_threaded = true` pour sortir de `updateServices()` (on gère son cycle soi-même via `cycle_manager.registerCycle`).

### 9.3 Ajouter un Block / Mission
1. `blockRegistry.add("collect_A", priority, score, estimatedMs, &action, &feasible);` dans `strategy.cpp::registerBlocks()`.
2. `action` renvoie un `BlockResult` (`SUCCESS`, `FAIL`, `ABANDONED`).
3. Pour décomposer en étapes : `Planner& p = ...; p.addMission("...").addStep(...).addStep(...).setMaxRetries(n);`.

### 9.4 Émettre/recevoir via Intercom (depuis T4.1)
```cpp
intercom.sendRequest("ob(0, 400)", 200, [](Request& r){
    if (strcmp(r.response(), "1") == 0) safety.setObstacleDetected(true);
});
```
Pour traiter côté T4.0, enregistrer un handler dans `onIntercomRequest(Request& req)` de `main.cpp`.

### 9.5 Lancer un mouvement
```cpp
async motion.setAbsolute();
async motion.noStall().feedrate(0.6f).go(1000, 500);   // non-bloquant
await os.wait();                                        // bloque jusqu'à fin
Console::info("Motion") << motion.getLastStats().durationMs << "ms" << Console::endl;
```
Rappel : `async` / `await` sont les keywords du mini-interpréteur `utils/interpreter`. En C++ pur, on utilise `motion.setAsync()` + `os.wait(job, true)`.

---

## 10. Deep-dive : contrôleurs de mouvement

Cette section détaille l'empilement des contrôleurs situé dans `teensy41/src/services/motion/`. C'est le cœur temps-réel du firmware — toute modification impacte directement le comportement du robot en piste.

### 10.1 Pipeline global

```
API utilisateur (motion.go / via / goPolar / aim / turn / goAlign)
        │
        ▼
Motion (service)                                    [main loop, ~500 Hz]
    ├─ file de waypoints (8 slots, rayon 80 mm)
    ├─ MoveOptions (feedrate, stall, snap, cancelOnStall…)
    └─ dispatch vers 1 des 3 contrôleurs :
        │
        ├── PositionController  (mode croisière, boucle fermée)    ◄── LEGACY_WAYPOINT + OTOS OK
        ├── PursuitController   (poursuite de cible vivante)      ◄── LIVE_PURSUIT
        └── StepperController   (profil trapézoïdal, boucle ouverte) ◄── OTOS KO / fallback
            │
            ▼
VelocityController (commun aux 2 premiers)
    ├─ consigne (vx, vy, vrot) dans le repère robot
    ├─ kinematics::ik() → consignes (va, vb, vc) steppers
    └─ normalisation à MAX_SPEED stepper (15000 steps/s)
        │
        ▼
3 × Stepper (instances A, B, C)                     [ISR T_10US, ~100 kHz]
    └─ digitalWriteFast sur pins step ± dir
```

Deux cycles d'horloge clés du `CycleManager` :
- `T_1MS → control()` appelle la boucle PID du contrôleur actif. Throttled en interne à `PID_INTERVAL = 2000 µs` → effectivement **500 Hz** pour la boucle de contrôle.
- `T_10US → step()` appelle la génération d'impulsions des 3 steppers. Chaque `Stepper::step()` décide individuellement (via `m_step_delay` calculé depuis la vitesse courante) s'il doit émettre un front.

### 10.2 Cinématique holonomique

`kinematics.cpp` définit deux fonctions pures, appelées à chaque cycle :

```
Vec3 ik(Vec3 target)   // Cartésien → Holonomique (steppers)
Vec3 fk(Vec3 target)   // Holonomique → Cartésien
```

Matrice géométrique P (3 roues à 120°, avec le bras de levier L) :

```
        [  0       1     L ]
P  =    [ -√3/2  -1/2    L ]
        [  √3/2  -1/2    L ]
```

Séquence appliquée dans `ik()` :
1. `target *= Calibration::Current.Cartesian` — correction d'échelle runtime (X, Y, rotation).
2. `target = P × target` — projection dans le repère des 3 roues.
3. `target *= Calibration::Current.Holonomic × STEP_MODE` — correction par roue et passage en micro-pas (`STEP_MODE = 8`).

`fk()` est l'inverse. La calibration est mutable à chaud via commandes terminal `calib_*`, ce qui permet d'ajuster en piste sans recompiler.

### 10.3 Motion (service)

**Fichier** : `services/motion/motion.{h,cpp}` (~294 lignes cpp, API dans le .h).

État interne principal :

| Variable | Type | Rôle |
|----------|------|------|
| `m_controlMode` | `ControlMode` | `LEGACY_WAYPOINT` ou `LIVE_PURSUIT` |
| `use_cruise_mode` | bool | Si `true` + OTOS enabled → PositionController, sinon StepperController |
| `current_move_cruised` | bool | Snapshot du contrôleur retenu pour le mouvement en cours |
| `current_move_pursuit` | bool | Vrai si PursuitController actif |
| `m_waypoints[8]` | Waypoint[] | File FIFO, capacité 8. Dépassement = log d'erreur et rejet |
| `m_pendingOpts` | MoveOptions | Options accumulées par l'API fluent, copiées à `go()` dans `m_activeOpts` |
| `m_feedrate` | float | Scale global [0.05, 1.0] clampé à l'entrée |
| `m_moveStartMs` | uint32_t | Démarrage du move pour la gestion de timeout |

**Fluent API** (`motion.h` l.128-142) :
```cpp
motion.withStall().cancelOnStall().feedrate(0.7f).via(200, 0).go(500, 300);
```
Chaque builder renvoie une référence sur `Motion&` et écrit dans `m_pendingOpts`. `go(x, y)` enfile un `Waypoint{target, passThrough=false, opts=m_pendingOpts}` puis, si aucun job n'est en cours, appelle `start()`.

`MoveOptions::withGlobalDefaults()` lit `RuntimeConfig` pour `motion.border_snap` et `motion.collision`. Quand `borderSnap` est actif, il force aussi `stallEnabled` et `cancelOnStall`.

**Waypoints** :
- Capacité : 8.
- `WAYPOINT_RADIUS = 80 mm` : un waypoint intermédiaire (`passThrough=true`) est considéré atteint dès qu'on entre dans ce rayon → on avance au suivant sans stopper (pour conserver la fluidité).
- Le dernier waypoint (non-passThrough) n'avance pas tant que `checkCompletion()` n'est pas satisfait.

**Timeout** : hardcodé à **5000 ms** dans `motion.cpp` (un TODO signale qu'il devrait venir de `RuntimeConfig`). À dépassement → `cancel()` sur le contrôleur actif + log d'erreur.

**Mode estimé** : `Motion::estimatedPosition()` renvoie `localisation.getPosition()` si OTOS enabled, sinon `_position + stepper_controller.getDisplacement()` (dead-reckoning).

**MoveStats** : enregistré à la fin de chaque mouvement. Contient `durationMs`, `targetDistMm`, `traveledMm`, `targetAngleDeg`, `traveledAngleDeg`, `stalled`, `stallChecks`, `stallMinTransMm`, `stallMinAngleDeg`, `satXCycles`, `satYCycles`, `satZCycles`. Consultable via `motion.getLastStats()` et `motion.printDiagReport()`.

### 10.4 PID (controller/pid.h)

Implémentation minimaliste, une par axe :

```cpp
class PIDController {
    float kP, kI, kD;
    float integral, prevError;
    float compute(float error, float dt, bool saturated = false) {
        if (!saturated) integral += error * dt;          // anti-windup "freeze"
        float derivative = (error - prevError) / dt;
        prevError = error;
        return kP * error + kI * integral + kD * derivative;
    }
    void reset();
};
```

Particularités :
- **Pas de filtre dérivée** : `D = (e - e_prev)/dt` brut → sensible au bruit de l'OTOS. C'est pourquoi `kD` est élevé (100) mais l'effet reste tolérable grâce à la cadence fixe (2 ms).
- **Anti-windup par freeze** : si la sortie est saturée, l'intégrale est gelée. Ici `kI = 0` partout, donc cela ne joue pas en pratique.
- **Pas de filtre passe-bas** en sortie.

### 10.5 PositionController (croisière, boucle fermée)

**Fichiers** : `controller/positionController.{h,cpp}` (~365 lignes cpp).

**Sélection** : `use_cruise_mode == true` **ET** `localisation.enabled() == true`. Sinon fallback vers StepperController.

**Gains PID hardcodés dans le constructeur** (pas lus depuis Settings) :

| Axe | kP | kI | kD |
|-----|----|----|----|
| vx | 4.0 | 0.0 | 100.0 |
| vy | 4.0 | 0.0 | 100.0 |
| vrot | 10.0 | 0.0 | 70.0 |

**Sequence à chaque cycle `control()` (throttled à 500 Hz)** :
1. Lecture odo : `pos = localisation.getPosition()`, `vel = localisation.getVelocity()`.
2. Calcul erreurs : `error.xy = target.xy - pos.xy`, `error.c = shortestAngleDiff(target.c, pos.c)`.
3. `computeVelocity(dt, error, angleErr)` :
   - 3 PID indépendants → sortie en vitesse cartésienne monde.
   - Ramp d'accélération limité à `MAX_ACCEL` / `MAX_ROT_ACCEL`.
   - Snap-to-zero si `|error| < MIN_DISTANCE` (20 mm) ou `|angleErr| < MIN_ANGLE` (≈2°).
   - Clamp sortie à `±MAX_SPEED`, `±MAX_ROT_SPEED` → met à jour les flags `m_satX/Y/Z` (et compteurs diagnostic).
4. APF : si `m_apfEnabled`, ajoute un gradient répulsif issu de la carte d'occupation (`occupancy.repulsiveGradient(pos2d) * m_apfScale`), puis re-clamp. **Appliqué après le check de complétion** pour ne pas bloquer l'arrivée.
5. Rotation repère monde → repère robot (matrice de rotation de `pos.c`) → obtient `cmd_robot`.
6. Stall detector : `updateVelocity(cmd_robot, vel, dt)` + `updateStagnation(pos, target, dt)` + legacy `update(pos, elapsedMs)`.
7. `velocity_controller.setTargetVelocity(cmd_robot)`.

**Critères de complétion** (`checkCompletion`) :
- vitesse → 0 **et**
- `|error.x| < MIN_DISTANCE` **et**
- `|error.y| < MIN_DISTANCE` **et**
- `|angleErr| < MIN_ANGLE` **et**
- job toujours en état `RUNNING`.

**Saturation & diagnostics** : `satXCycles`, `satYCycles`, `satZCycles` incrémentés à chaque cycle où la sortie PID clippe. Visibles dans `MoveStats`.

**Border snap** (actuellement commenté dans le code, à réactiver) : si stall détecté à < 150 mm d'un mur → `snapAxisTarget(axis, valeur_connue)` + clear du flag stall sur cet axe. Sinon (collision objet) → `cancel()`.

### 10.6 PursuitController (poursuite vivante, mode LIVE_PURSUIT)

**Fichiers** : `controller/pursuitController.{h,cpp}` (~263 lignes).

**Sélection** : `m_controlMode == LIVE_PURSUIT`. Activé implicitement par la Jetson via les commandes `aim(x,y)` à ~20 Hz.

**Différences clés avec PositionController** :
- Pas de complétion automatique : seul `motion.cancel()` arrête le mouvement (l'hôte décide quand on est arrivé).
- Pas de décélération near-goal : c'est la carotte (côté Python) qui gère la fin.
- `setTarget(Vec2)` (interne) / `aim(Vec2)` (API) met à jour la cible sans réinitialiser les PID.
- **Watchdog 200 ms** : si pas de nouvel `aim()` reçu → `m_watchdogTripped = true`, et `velocity *= 0.85` à chaque cycle (freinage exponentiel). Dès qu'un `aim()` arrive, watchdog dégelé.
- Mêmes gains PID que PositionController.

**Heading mode** (`setHeadingMode(true, RobotCompass face)`) :
- À chaque cycle :
  ```
  dir = target - position
  si |dir| < 10 mm → garde l'orientation courante (évite les oscillations en fin de trajet)
  sinon :
      heading_world = atan2(dir.y, dir.x)
      target.c = heading_world − getCompassOrientation(face)
  ```
- Permet d'orienter automatiquement une face du robot (A, B, C…) vers la direction de déplacement.

### 10.7 StepperController (trapezoïdal, boucle ouverte)

**Fichiers** : `controller/stepperController.{h,cpp}` (~351 lignes).

**Sélection** : fallback quand `use_cruise_mode == false` ou OTOS non dispo. Appelé aussi en pré-match pour les séquences de recalage.

**Profil trapézoïdal** via `LinStepAccelerator` :
```cpp
void prepare(currentPos, targetPos, vTarget, vStart, vEnd, accel);
float updateSpeed(currentPos);   // renvoie la vitesse instantanée
```
Avec les piecewise :
- si `s < accEnd` → `v = sqrt(2·a·s + vStart²)`
- si `s < decStart` → `v = vTarget`
- sinon → `v = sqrt(2·a·(ds − s − 1) + vEnd²)`

**Constantes steppers** (de `settings.h`) :
- `MAX_SPEED = 15000 steps/s`
- `PULL_IN = PULL_OUT = 100 steps/s` (vitesse plancher d'accroche/décroche)
- `MAX_ACCEL = STOP_DECCEL = 3000 steps/s²`

**Synchronisation lead-stepper (Bresenham)** :
Les 3 steppers font des distances différentes. Le "lead" (plus grande distance) impose la cadence ; les 2 autres avancent selon un algo Bresenham mid-point :
```cpp
// Init : error = 2·delta - leadDelta
// Lead step :
leadStepper->step(); stepsDone++;
for (autres s) {
    s->error += 2 * s->delta;
    if (s->error >= leadDelta) {
        s->step();
        s->stepsDone++;
        s->error -= 2 * leadDelta;
    }
}
```
Résultat : trajectoire linéaire cohérente sans PID, tant qu'aucun stepper ne saute de pas.

### 10.8 VelocityController (pont IK → pulses)

**Fichiers** : `controller/velocityController.{h,cpp}`.

Appelé par PositionController et PursuitController pour appliquer la consigne (vx, vy, vrot) :
1. `target = ik(targetVelocity)` → consignes (va, vb, vc) dans le repère holo.
2. `M = max(|va|, |vb|, |vc|)` ; si `M > MAX_SPEED stepper` → scale `target *= MAX_SPEED / M` (conservation du ratio, donc de la direction).
3. `stepper[i].setVelocity(target[i])` pour chaque roue.

`Stepper::setVelocity()` convertit en période `m_step_delay = 1e6 / |v|` (µs/step), clampée à `MIN_STEP_DELAY = 20 µs`. Si `|v| < 1e-4` → `m_step_delay = 0` (stepper muet).

### 10.9 Stepper (génération de fronts)

**Fichiers** : `services/motion/stepper.{h,cpp}`.

API publique principale :
```cpp
void setDirection(bool forward);
void setVelocity(float vel);      // clampe, met à jour m_step_delay
void step();                      // emit un front si delay écoulé et |v| ≥ PULL_IN
bool shouldStep();                // test non-destructif
void enable() / disable();
long position() const;
```

Dans la boucle ISR `T_10US` (fonction globale `step()` dans `main.cpp` / `routines.cpp`) :
```cpp
for (each stepper) if (stepper.shouldStep()) stepper.step();
```

Génération du pulse : `digitalWriteFast(HIGH) ; delayMicroseconds(PULSE_WIDTH=14 µs) ; digitalWriteFast(LOW)`. La position logique `m_position` est incrémentée de ±1 selon le signe de `m_velocity`.

### 10.10 StallDetector (3 méthodes en parallèle)

**Fichiers** : `services/motion/stallDetector.{h,cpp}` (~254 lignes).

Toutes les méthodes tournent en parallèle dans `PositionController::onUpdate()`. La première qui déclenche lève `m_stalledFlag`.

1. **Velocity mismatch** (`updateVelocity`) — réactif (~200 ms).
   - Par axe : si `|cmdVel| > velThresholdMms` et `|otosVel| < velThresholdMms` → accumule.
   - Déclenche quand accumulation > `velStallTimeS` (0.2 s).
   - Flags par axe : `stalledX`, `stalledY`, `stalledRot` — clearables indépendamment (utile pour le border snap).

2. **Error stagnation** (`updateStagnation`) — ~400 ms.
   - Snapshot d'erreur PID toutes les 100 ms.
   - Si l'erreur ne décroît pas de `stagMoveMm` (0.5 mm) pendant ≥ `stagTimeS` (0.4 s) → déclenche.
   - Rattrape les stall basse vitesse qui échappent au velocity mismatch.

3. **Displacement legacy** (`update`) — lent (~1.5 s).
   - Après `delayMs` (1000 ms) depuis le début du move.
   - Fenêtre de `periodMs` (500 ms) : si déplacement cumulé < `transDispMm` (5 mm) ou rotation cumulée < `angleDispRad` (0.02 rad) → déclenche.
   - Garde-fou final, peu discriminant mais robuste.

Tous les seuils proviennent de `Settings::Motion::Stall` et peuvent être surchargés par move via `motion.withStall().config.XXX = ...`.

### 10.11 Grammaire des erreurs / failure modes

| Scenario | Comportement |
|----------|--------------|
| OTOS déconnecté en cruise | Basculement vers stepper dead-reckoning ; tracking dégradé |
| Stepper saute un pas sous charge | Velocity mismatch déclenche → cancel ou snap selon options |
| Timeout 5 s dépassé | `cancel()` contrôleur, log d'erreur, move échec |
| Overflow file de waypoints | Log d'erreur, nouveau waypoint rejeté (les précédents gardent la priorité) |
| Watchdog Pursuit (> 200 ms sans aim) | freinage exponentiel `v *= 0.85` par cycle, reste actif |
| Feedrate négatif/hors range | Clampé à [0.05, 1.0] à l'entrée |

### 10.12 Cheat-sheet des valeurs

| Quoi | Valeur | Source |
|------|--------|--------|
| Fréquence PID | 500 Hz (2 ms) | `PID_INTERVAL` + `T_1MS` |
| Fréquence ISR step | 100 kHz (10 µs) | `T_10US` |
| Rayon waypoint pass-through | 80 mm | `WAYPOINT_RADIUS` |
| Tolérances position/angle | 20 mm / 2° | `MIN_DISTANCE`, `MIN_ANGLE` |
| Max speed cartésien / rot | 3800 mm/s / 10 rad/s | `Settings::Motion` |
| Max accel cartésien / rot | 3500 mm/s² / 30 rad/s² | idem |
| PID gains (x, y) | 4, 0, 100 | hardcodé dans le constructeur |
| PID gains (rot) | 10, 0, 70 | hardcodé |
| Watchdog Pursuit | 200 ms | PursuitController |
| Timeout move | 5000 ms | hardcodé `motion.cpp` (TODO) |
| Stall velocity mismatch | 0.2 s | `Stall::velStallTimeS` |
| Stall stagnation | 0.4 s | `Stall::stagTimeS` |
| Stall displacement legacy | 1 s + 500 ms | `DELAY_MS`, `PERIOD_MS` |
| Min step delay | 20 µs | `MIN_STEP_DELAY` |
| Pulse width | 14 µs | `PULSE_WIDTH` |

---

## 11. SparkFun OTOS (Optical Tracking Odometry Sensor)

Le robot utilise le **SparkFun Optical Tracking Odometry Sensor — PAA5160E1 (Qwiic)** (SKU `SEN-24904`) comme unique source d'odométrie absolue. Il remplace une centrale inertielle + encodeurs tout en donnant directement une pose `(x, y, θ)` dans un repère monde. Le capteur embarque son propre micro-contrôleur qui fait la fusion PAA + IMU et retourne un état filtré (Kalman) en I²C.

### 11.1 Composants physiques

- **PAA5160E1** (PixArt Imaging) — capteur de flux optique laser **Class 1, 850 nm**.
  - Résolution **20 000 DPI**, cadence **20 000 fps**.
  - Vitesse de tracking jusqu'à **2.5 m/s**, erreur typique 3–5 %.
  - Plage de distance capteur-sol **10–27 mm** (à respecter en fixation châssis).
  - Surfaces OK : béton, époxy, bois stratifié, plancher lustré ou semi-lustré. Surfaces KO : verre, miroir, tapis épais.
- **LSM6DSO** (STMicroelectronics) — IMU 6 axes intégré : accéléromètre ±2/±4/±8/±16 g, gyro ±125/±250/±500/±1000/±2000 dps. Utilisé par la fusion interne pour rattraper les micro-décrochages optiques.
- **MCU embarqué** sur la carte OTOS : exécute la fusion capteur + Kalman et expose les résultats via I²C (pas de calcul à faire côté Teensy).
- **Connectique** : 2 × Qwiic (JST 4 broches) + 6 PTH (3.3 V, GND, SDA, SCL + 2 IO dont `IO9` = "data ready" interrupt).
- **Adresse I²C par défaut** : `0x17`.
- **Alimentation** : 3.3 V via Qwiic (consommation quelques dizaines de mA).

### 11.2 Ranges internes du capteur (rail int16)

Le capteur communique en int16 signés ; les facteurs de conversion sont fixes :

| Grandeur | Plage utile | Facteur int16 |
|----------|-------------|---------------|
| Position linéaire | ±10 m | 32768 / 10 |
| Vitesse linéaire | ±5 m/s | 32768 / 5 |
| Accélération | ±16 g | 32768 / (16·9.80665) |
| Angle | ±π rad | 32768 / π |
| Vitesse angulaire | ±2000 dps | 32768 / (2000·π/180) |
| Accélération angulaire | ±180 000 dps² | 32768 / (π·1000) |

Donc : la plage maxi mesurable côté vitesse est **5 m/s**, bien au-delà du tracking garanti (2.5 m/s). Important pour les saturations : `MAX_SPEED = 3800 mm/s` du firmware est compatible mais hors spec tracking optique.

### 11.3 API Arduino (`sfDevOTOS` / `QwiicOTOS`)

La lib `SparkFun_Qwiic_OTOS_Arduino_Library` expose une classe `QwiicOTOS` dérivant de `sfDevOTOS`. Méthodes publiques principales (toutes renvoient `sfTkError_t` sauf indiqué) :

```cpp
// Connexion
bool        begin(TwoWire& wirePort = Wire);   // overload dans QwiicOTOS
sfTkError_t isConnected();
sfTkError_t getVersionInfo(sfe_otos_version_t& hw, sfe_otos_version_t& fw);
sfTkError_t selfTest();

// Calibration
sfTkError_t calibrateImu(uint8_t numSamples = 255, bool waitUntilDone = true);
sfTkError_t getImuCalibrationProgress(uint8_t& numSamples);

// Unités
sfe_otos_linear_unit_t  getLinearUnit();                       // kSfeOtosLinearUnitMeters / Inches
void                    setLinearUnit(sfe_otos_linear_unit_t);
sfe_otos_angular_unit_t getAngularUnit();                      // kSfeOtosAngularUnitRadians / Degrees
void                    setAngularUnit(sfe_otos_angular_unit_t);

// Scalars (plage IMPORTANTE : 0.872 à 1.127)
sfTkError_t getLinearScalar(float& s);
sfTkError_t setLinearScalar(float s);       // kMinScalar = 0.872f, kMaxScalar = 1.127f
sfTkError_t getAngularScalar(float& s);
sfTkError_t setAngularScalar(float s);      // même plage

// Offset (montage hors-centre)
sfTkError_t getOffset(sfe_otos_pose2d_t& pose);
sfTkError_t setOffset(sfe_otos_pose2d_t& pose);

// Pose / dérivées
sfTkError_t resetTracking();
sfTkError_t getPosition(sfe_otos_pose2d_t& pose);
sfTkError_t setPosition(sfe_otos_pose2d_t& pose);
sfTkError_t getVelocity(sfe_otos_pose2d_t& pose);
sfTkError_t getAcceleration(sfe_otos_pose2d_t& pose);

// Kalman StdDev
sfTkError_t getPositionStdDev(sfe_otos_pose2d_t& pose);
sfTkError_t getVelocityStdDev(sfe_otos_pose2d_t& pose);
sfTkError_t getAccelerationStdDev(sfe_otos_pose2d_t& pose);

// Burst reads (1 transaction I²C)
sfTkError_t getPosVelAcc(pos, vel, acc);
sfTkError_t getPosVelAccStdDev(posStd, velStd, accStd);
sfTkError_t getPosVelAccAndStdDev(pos, vel, acc, posStd, velStd, accStd);

// Signal processing + status
sfTkError_t getSignalProcessConfig(sfe_otos_signal_process_config_t& cfg);   // enLut, enAcc, enRot, enVar
sfTkError_t setSignalProcessConfig(sfe_otos_signal_process_config_t& cfg);
sfTkError_t getStatus(sfe_otos_status_t& st);                                // warnTiltAngle, warnOpticalTracking, errorPaa, errorLsm
```

Structure pose :
```cpp
struct sfe_otos_pose2d_t { float x, y, h; };   // h = heading
```

Constantes utiles :
- `kDefaultAddress = 0x17`
- `kMinScalar = 0.872f`, `kMaxScalar = 1.127f`
- `kMeterToInch = 39.37f`, `kRadianToDegree = 180/π`

### 11.4 API Python (`qwiic_otos`)

Package pip : `pip3 install sparkfun-qwiic-otos`. Classe principale `QwiicOTOS`. Compatible **Python standard / MicroPython / CircuitPython**. Méthodes miroir de l'API Arduino + helpers `setUnitsMm()` / `setUnitsInches()`.

Exemple officiel :
```python
import qwiic_otos, time
otos = qwiic_otos.QwiicOTOS()
otos.begin()
otos.calibrateImu()
otos.resetTracking()
while True:
    pos = otos.getPosition()
    print(f"X={pos.x:.3f}  Y={pos.y:.3f}  H={pos.h:.2f}")
    time.sleep(0.5)
```

Utile pour tests hors-ligne depuis la Jetson via Qwiic breakout si besoin de déboguer le capteur indépendamment du Teensy.

### 11.5 Intégration dans le firmware T4.1

Le service `Localisation` (`services/localisation/localisation.{h,cpp}`) enveloppe `QwiicOTOS` :

- **À `attach()`** : `begin(Wire)`, `setLinearUnit(Meters)`, `setAngularUnit(Radians)`, charge `Calibration::Current.OTOS_Linear` → `setLinearScalar()` et `OTOS_Angular` → `setAngularScalar()`. Ces scalaires **doivent être dans [0.872, 1.127]** — sinon la lib renvoie une erreur et la calibration n'est pas appliquée.
- **`calibrate()`** : robot immobile → `calibrateImu(numSamples=255, waitUntilDone=true)`. Dure ~2.5 s. Appelé dans `onRobotBoot()` entre `motion.engage()` et `motion.disengage()` pour être sûr que les steppers n'introduisent pas de micro-vibration.
- **Boucle lecture** : `getPosition()` / `getVelocity()` dans `PositionController::onUpdate()` et `PursuitController::onUpdate()` à 500 Hz. Côté firmware, les valeurs sont converties en **mm** (via le scalar) et **radians**.
- **`setPosition(Pose2D)`** : utilisé par le recalage en début de match (force la pose au coin de la table d'équipe) et par le border snap quand il est réactivé.
- **`getStatus()`** : non exploité pour l'instant, mais les flags `warnOpticalTracking` / `errorPaa` / `errorLsm` pourraient être câblés sur le canal `T:cal` pour diagnostic.
- **`getPositionStdDev()`** : idem, non exploité mais pourrait moduler les gains PID quand l'incertitude explose.

Le service expose aussi `useIMU()` (true ssi connecté + calibré). Si false, Motion bascule automatiquement en StepperController pour tout nouveau mouvement.

**Discrépance doc/code à lever** : la doc annonce `scalar = 0.990723`, le code use `1.022`. Les deux sont dans la plage valide mais ne sont pas équivalents. À trancher (probablement suite à une re-calibration physique) et à figer dans `Calibration::Current`.

### 11.6 Conventions de repère choisies côté holOS

- Linéaire : **millimètres** (après multiplication par 1000 du mètre natif OTOS).
- Angulaire : **radians** (convention C++), converti en **milliradians** dans la télémétrie `T:a` pour compresser.
- Origine : définie par `setPosition()` au recalage de début de match (coin de table d'équipe, typiquement `startYellow` ou `startBlue` de `poi.h`).
- X vers la droite de la table (3000 mm), Y vers le bas (2000 mm). θ trigo standard (référence = face A du robot côté firmware, converti vers "0 = est" côté holOS via `theta_offset_deg`).

### 11.7 Checklist procédure de calibration

1. Poser le robot à plat sur le sol cible (pas sur l'établi).
2. Laisser immobile, alimentation stable.
3. Commande terminal `calib_start` → déclenche `localisation.calibrate()`.
4. Attendre ~2.5 s (255 échantillons IMU, 100 ms par sample).
5. Scalars : utiliser l'exemple officiel "Calibration" (SparkFun) pour trouver les coefficients linéaire et angulaire propres au châssis → écrire dans `Calibration::Current` via commande `calib_linear <val>` / `calib_angular <val>`. Les deux doivent rester dans **[0.872, 1.127]**.
6. Offset de montage : si l'OTOS n'est pas au centre géométrique du robot, renseigner l'offset via `setOffset()` (actuellement non exposé côté terminal — TODO à ajouter).

### 11.8 Documentation officielle et ressources

- Page produit : https://www.sparkfun.com/sparkfun-optical-tracking-odometry-sensor-paa5160e1-qwiic.html
- Hookup guide complet : https://docs.sparkfun.com/SparkFun_Optical_Tracking_Odometry_Sensor/
- Vue "single page" : https://docs.sparkfun.com/SparkFun_Optical_Tracking_Odometry_Sensor/single_page/
- Hardware overview : https://docs.sparkfun.com/SparkFun_Optical_Tracking_Odometry_Sensor/hardware_overview/
- Exemples Arduino : https://docs.sparkfun.com/SparkFun_Optical_Tracking_Odometry_Sensor/examples/
- Setup Python : https://docs.sparkfun.com/SparkFun_Optical_Tracking_Odometry_Sensor/software_setup-Python/
- Exemples Python/XRP : https://docs.sparkfun.com/SparkFun_Optical_Tracking_Odometry_Sensor/examples_xrp/
- Hardware assembly FTC : https://docs.sparkfun.com/SparkFun_Optical_Tracking_Odometry_Sensor/hardware_assembly_FTC/
- Librairie Arduino (GitHub) : https://github.com/sparkfun/SparkFun_Qwiic_OTOS_Arduino_Library
- Arduino API reference (Doxygen) : https://docs.sparkfun.com/SparkFun_Qwiic_OTOS_Arduino_Library/
- Librairie Python (GitHub) : https://github.com/sparkfun/qwiic_otos_py
- Package pip : https://pypi.org/project/sparkfun-qwiic-otos/
- Hardware repo (schéma Eagle) : https://github.com/sparkfun/SparkFun_Optical_Tracking_Odometry_Sensor
- Forum communauté : https://community.sparkfun.com/t/sparkfun-optical-tracking-odometry-sensor-paa5160e1-qwiic/47654

### 11.9 Limitations connues

- **Sol brillant / transparent** → décrochage possible (voir "warnOpticalTracking" dans status).
- **Vibrations / chocs** : la fusion interne supporte mal les chocs ; un crash peut nécessiter un `resetTracking()` + `setPosition()`.
- **Vitesse > 2.5 m/s** : dérive rapide. `MAX_SPEED = 3800 mm/s` dépasse la spec tracking → en pratique, `feedrate < 0.65` recommandé si on veut rester dans les clous.
- **Hauteur capteur** : respecter **10–27 mm** par rapport au sol (choix final vers 15 mm pour garder de la marge face aux déformations du plancher de la table).
- **Pas de redondance** : si l'OTOS se déconnecte, on retombe sur dead-reckoning stepper → dérive cumulative très rapide (pas d'encoder externe).
- **Scalar hors [0.872, 1.127]** : rejeté silencieusement par la lib → toujours vérifier la valeur retournée par `getLinearScalar()` après écriture.
- **Tilt** : inclinaison trop forte (signalée par `warnTiltAngle`) → optical tracking dégradé.

---

## 12. API Python (holOS, partiellement fonctionnelle)

Le firmware T4.1 n'est qu'une moitié du système. L'autre moitié — la stratégie haut-niveau et le replanning — tourne côté Jetson (ou PC en mode simulation) dans le dossier `holOS/software/`. Cette API est signalée par l'auteur comme **partiellement fonctionnelle** : les chemins "heureux" marchent, mais certaines options de mouvement (goAlign, cancel_on_collide, certaines séquences de fallback) ne sont pas encore câblées de bout en bout.

### 12.1 Structure côté Python (`holOS/software/`)

```
brain.py                      ← orchestrateur principal (Brain class)
run.py                        ← entry point PC + Jetson (sim, wired, xbee)
holos_cli.py                  ← CLI interactive
services/
    motion.py                 ← MotionService (miroir Python de Motion C++)
    safety.py, chrono.py, actuators.py, occupancy.py, vision.py
shared/
    protocol.py               ← framing CRC8-SMBUS identique au firmware
    config.py, settings.py, occupancy.py, pathfinder.py (A*)
transport/
    base.py, wired.py, xbee.py, virtual.py  (VirtualTransport pour la simu)
strategy/
    match.py                  ← script stratégie (hot-reloadable)
```

### 12.2 Protocole fil (`shared/protocol.py`)

Framing identique à l'Intercom embarqué :
```
Requête  : <uid>:<content>|<crc8>\n
Réponse  : r<uid>:<response>|<crc8>\n
Télémétrie : TEL:<type>:<data>|<crc8>\n
Messages simples : ping\n , pong:<bridge>\n
```
CRC8-SMBUS (polynôme 0x07, init 0x00) via `crcmod.predefined.mkCrcFun('crc-8')`. Cohérent avec `FastCRC.smbus()` côté Teensy.

**Commandes supportées (Jetson → Teensy)** — tirées du docstring `shared/protocol.py` :
```
hb                              heartbeat (reply "ok")
tel                             snapshot télémétrie
occ                             dump occupancy map
go(x, y)                        move absolu
goPolar(angle, dist)
turn(angle)                     rotation absolue
align(side, angle)              alignement face robot ↔ orientation
setAbsPosition(x, y, angle)     force la pose
cancel / pause / resume
elevator(side, pose) / grab(side) / drop(side)
start / stop
fb(id)                          déclenche un fallback
feed(feedrate)                  set feedrate [0.05, 1.0]
```

**Télémétrie compacte (Teensy → Jetson)** — format compressé par rapport à la doc historique :
```
T:a  px py θ R tx ty dist feed safety chrono     aggregated MOVING
T:a  px py θ I feed safety chrono                aggregated IDLE  (10 Hz)
T:m  DONE:ok|fail,dur=…,dist=…,stall=0|1         motion done
T:od gx,gy;gx,gy;…                               occupancy map sparse
T:mask 11110                                     canaux actifs
T:cal key=val key=val …                          rapport calibration
```
Les formats `TEL:pos:…`, `TEL:motion:…`, `TEL:safety:…`, `TEL:chrono:…` existent encore en legacy mais sont dépréciés.

### 12.3 MotionService Python (mode LIVE_PURSUIT)

`services/motion.py` expose deux modes :
- **`LEGACY_WAYPOINT`** : envoie `via(); via(); go()` par commande A*. Robuste, mais saccadé car le contrôleur firmware reset l'accélération à chaque waypoint.
- **`LIVE_PURSUIT`** : envoie `aim(x, y)` à **20 Hz** (`PURSUIT_TICK_S = 0.05`). Une "carotte" glisse sur le chemin A* avec les paramètres :
  - `PURSUIT_CARROT_DIST = 300 mm` — distance d'avance de la carotte.
  - `PURSUIT_REPLAN_EVERY = 10 ticks` — replan A* toutes les ~500 ms.
  - `PURSUIT_LOCK_FACTOR = 1.2` — dès que `dist_to_goal < CARROT × LOCK` → on verrouille la carotte sur la cible finale.
  - `PURSUIT_ARRIVED_DIST = 30 mm` — seuil d'arrivée.
  - `PURSUIT_TIMEOUT_S = 30 s` — timeout sécurité.

La fréquence de mise à jour (20 Hz) doit rester supérieure à la fréquence du watchdog firmware (`> 1 / 200 ms = 5 Hz`) — condition vérifiée.

### 12.4 Replanning réactif

Sur échec de mouvement (`motion_timeout` ou `stall`), si `use_pathfinding=True` et qu'un `SafetyService` est attaché :
- Cancel du move courant.
- Attente **`REPLAN_DELAY_S = 1.5 s`** (laisse à l'adversaire le temps de s'écarter).
- Replan A* sur la grille d'occupation dynamique.
- Nouvel envoi. Max **`MAX_REPLANS = 3`** par `go()`.

### 12.5 Offset d'angle `theta_offset_deg`

Le firmware et holOS n'utilisent pas le même zéro d'angle :
- holOS : 0 = est (convention math trigonométrique).
- Firmware : 0 = face A du robot.

`theta_offset_deg` (default 0 en sim, `HW_THETA_OFFSET_DEG` en hardware) fait la conversion entre les deux repères avant envoi et après réception.

### 12.6 Transports

- `VirtualTransport` — simulateur pur, couplé à un `SimBridge` qui reproduit la physique holonomique.
- `WiredTransport` — USB-CDC, `/dev/ttyACM0` ou `COMx`.
- `XBeeTransport` — radio 868 MHz, sur Jetson : `/dev/ttyTHS1` (UART2 interne, pins 8 & 10 du header 40-pin), baud 57600.

Auto-détection côté `run.py` : Linux → suppose Jetson, auto-connect `/dev/ttyUSB0` ou `/dev/ttyTHS1` ; Windows → mode idle/sim par défaut.

### 12.7 État de maturité / points connus

- Les commandes `goAlign`, `cancel_on_collide`, `borderSnap` sont partiellement supportées : les flags sont transmis mais certains chemins de code firmware les ignorent (cohérent avec les TODO Actuators/borderSnap).
- Le mode `LIVE_PURSUIT` Python est le plus abouti ; c'est celui utilisé en compétition.
- La simulation `VirtualTransport + SimBridge` réplique la physique assez bien pour dev la stratégie, mais ne simule pas encore la dérive OTOS ni les stall mur.
- Le hot-reload de `strategy/match.py` fonctionne pendant un match (utile pour itérer sans couper la Jetson).

---

## 13. Ressources externes

- TeensyThreads : https://github.com/ftrias/TeensyThreads
- TwinsyStep (fork TeensyStep) : https://github.com/LesKaribous/TwinsyStep
- Qwiic OTOS (Arduino lib) : https://github.com/sparkfun/SparkFun_Qwiic_OTOS_Arduino_Library
- Qwiic OTOS API reference : https://docs.sparkfun.com/SparkFun_Qwiic_OTOS_Arduino_Library/
- OTOS hookup guide : https://docs.sparkfun.com/SparkFun_Optical_Tracking_Odometry_Sensor/
- ILI9341_t3 (Teensy 4.x) : https://github.com/PaulStoffregen/ILI9341_t3
- FastCRC : https://github.com/FrankBoesing/FastCRC
- Adafruit PCA9685 : https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library
- NeoPixel : https://github.com/adafruit/Adafruit_NeoPixel
- U8g2 : https://github.com/olikraus/u8g2
- crcmod (Python) : https://pypi.org/project/crcmod/
- Coupe de France de Robotique : https://www.coupederobotique.fr/
