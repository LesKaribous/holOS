# holOS Firmware Documentation

## Overview

The holOS firmware runs on two Teensy microcontrollers working in tandem inside a holonomic robot platform designed for the Coupe de France de Robotique. The Teensy 4.1 serves as the main controller (motion, strategy, actuators, remote control) while the Teensy 4.0 handles perception (LIDAR, obstacle mapping, visual feedback).

Both boards communicate via a hardwired UART intercom link at 31250 baud. The T4.1 can also be controlled remotely by a Jetson or PC via XBee radio or USB-CDC (57600 baud by default, configurable).

---

## Architecture at a Glance

```
                     XBee 868MHz              UART Intercom
  Jetson/PC  <-------------------->  T4.1  <---------------->  T4.0
  (holOS)       or USB-CDC           Main Controller            Perception
                                     - Motion (3 steppers)      - LIDAR LD06
                                     - Strategy/Mission         - Occupancy map
                                     - Actuators (servos,       - NeoPixel ring
                                       pumps, elevators)        - OLED display
                                     - Safety / Chrono
                                     - JetsonBridge
```

---

## Teensy 4.1 - Main Controller

### Service Architecture

All functionality is encapsulated in **services** inheriting from a common `Service` base class. Each service implements `attach()` (called once at registration) and `run()` (called every main loop iteration). Services are singletons registered with the OS via `os.attachService(&instance)`.

Registered services (by `ServiceID`):

| ID | Service | Role |
|----|---------|------|
| `ID_MOTION` | Motion | Holonomic movement with PID and stepper control |
| `ID_SAFETY` | Safety | LIDAR-based obstacle detection |
| `ID_CHRONO` | Chrono | Match timer (100s countdown) |
| `ID_ACTUATORS` | Actuators | Servos, pumps, elevators via PCA9685 |
| `ID_LIDAR` | Lidar | T4.1-side LIDAR data relay (via intercom) |
| `ID_VISION` | Vision | Camera/detection integration |
| `ID_IHM` | IHM | Buttons, switches, starter, buzzer |
| `ID_INTERCOM` | Intercom | T4.1 <-> T4.0 UART communication |
| `ID_JETSON` | JetsonBridge | Remote control from Jetson/PC |
| `ID_TERMINAL` | Terminal | Serial console commands |
| `ID_LOCALISATION` | Localisation | OTOS odometry sensor |
| `ID_NEOPIXEL` | Pixel | NeoPixel LED indicators |

### OS State Machine

```
BOOT
  |
  v
MANUAL_PROGRAM  <-- preparation phase (team selection, recalibration)
  |                  waits for starter insertion then removal
  v
AUTO_PROGRAM    <-- match start
  |                  runs mission planner or remote control loop
  v
STOPPED         <-- match end (100s elapsed)
```

State transitions:

- `BOOT -> MANUAL_PROGRAM`: After `onRobotBoot()` completes service attachment.
- `MANUAL_PROGRAM -> AUTO_PROGRAM`: Starter pin pulled (match begins).
- `AUTO_PROGRAM -> STOPPED`: Chrono fires match-end callback.
- Any state can be forced via `start` / `stop` terminal commands.

### Timing and Cycles

The firmware uses a threaded cycle manager spawning dedicated threads for time-critical loops:

| Cycle | Period | Function | Purpose |
|-------|--------|----------|---------|
| `T_10US` | 10 us | `step()` | Stepper pulse generation (ISR-level) |
| `T_1MS` | 1 ms | `control()` | PID velocity controller |
| Main loop | ~2 ms | `os.run()` | Service updates, command processing |

The stepper ISR at 10 us is the highest priority timing constraint. It generates precisely-timed step pulses for the three stepper motors. The PID controller runs at 1 ms (configured as `PID_INTERVAL = 2000 us` in settings, running on `T_1MS` cycle) and converts position errors into velocity setpoints.

### Boot Sequence

```
setup()
  1. Console::init() - serial output at CONSOLE_BAUDRATE
  2. Register routines (BOOT, MANUAL, AUTO, STOPPED, etc.)
  3. Register cycles: T_10US -> step(), T_1MS -> control()
  4. cycle_manager.start() - spawn threads

loop()
  os.run() -> dispatches to current state routine

onRobotBoot()
  1. Attach IHM (buttons, switches)
  2. Attach Actuators (PCA9685 servos, pumps)
  3. Attach Motion (stepper + OTOS init)
  4. Attach Chrono (match timer + callbacks)
  5. Attach Safety (disabled until match)
  6. Engage motion, calibrate OTOS, disengage
  7. Attach Intercom (T4.0 link + callbacks)
  8. Attach JetsonBridge (remote control + fallbacks)
  9. Attach Vision, Localisation, Pixel
  10. Register all terminal commands
  11. Transition to MANUAL_PROGRAM
```

### Motion System

The robot uses a **holonomic drive** with 3 stepper motors arranged at 120-degree intervals, controlled by two layered controllers:

**PositionController (cruise mode)** - Closed-loop PID using OTOS odometry feedback.

Three independent PID loops:
- `vx`: P=4.0, I=0.0, D=100.0
- `vy`: P=4.0, I=0.0, D=100.0
- `vrot`: P=10.0, I=0.0, D=70.0

The position controller converts (x, y, theta) errors into wheel velocities via holonomic inverse kinematics, then the stepper controller executes those velocities.

**StepperController (open-loop mode)** - Time-based trajectory planning with trapezoidal velocity profiles (LinStepAccelerator). Used for simple moves without feedback.

**Motion parameters** (from `settings.h`):

| Parameter | Value |
|-----------|-------|
| Max speed | 2800 mm/s | 
| Max acceleration | 500 mm/s^2 |
| Max rotation speed | 10 rad/s |
| Max rotation accel | 30 rad/s^2 |
| XY tolerance | 20 mm |
| Angle tolerance | 2 degrees |
| Steps/rev | 200 |
| Microstepping | 8x |
| Min step delay | 20 us |
| Pulse width | 14 us |

**Stall detection**: A sliding-window displacement analysis checks whether the robot has moved at least 5 mm (translational) or 0.02 rad (rotational) within each 500 ms window. Stall checking begins 1 s after movement start. If stall is detected, the movement can optionally be cancelled.

**Movement statistics**: Every completed movement records `MoveStats` (duration, distance traveled, stall info, saturation cycles, success flag). `motion.printDiagReport()` outputs a formatted summary.

### JetsonBridge (Remote Control)

The JetsonBridge service manages the communication with the Jetson (via XBee) or PC (via USB-CDC). It handles command dispatch, telemetry emission, heartbeat monitoring, and fallback management.

**Heartbeat**: Remote side must send `hb` commands at regular intervals. If no heartbeat arrives within 2000 ms, the bridge triggers `FallbackID::STOP`.

**Telemetry channels** (emitted periodically when enabled):

| Channel | Rate | Format |
|---------|------|--------|
| `pos` | 10 Hz | `TEL:pos:x=...,y=...,theta=...` |
| `motion` | 10 Hz | `TEL:motion:RUNNING/IDLE,tx=...,ty=...` |
| `safety` | 10 Hz | `TEL:safety:0/1` |
| `chrono` | 10 Hz | `TEL:chrono:milliseconds` |
| `occ` | 2 Hz | `TEL:occ:compressed_hex_data` |

Each telemetry frame is CRC8-SMBUS framed: `TEL:channel:data|crc\n`

**Motion completion**: When a motion command finishes, the bridge emits `TEL:motion:DONE:ok|crc` or `TEL:motion:DONE:fail|crc`.

### Intercom (T4.1 <-> T4.0)

Physical layer: Serial1 at 31250 baud (hardwired UART).

Message framing: `uid:content|crc8\n` for requests, `ruid:content|crc8\n` for replies. Plain messages: `content\n`.

Connection handshake: T4.1 sends `ping\n`, T4.0 replies `pong\n`. Connection lost if no data for 2 seconds.

Messages T4.1 sends to T4.0:
- `pos(x,y,theta)` - Update robot position for LIDAR calculations
- `ob(angle,distMax)` - Query obstacle presence at heading
- `gD(angle)` - Get raw distance at angle
- `oM` - Request compressed occupancy map
- `team(B|Y)` - Set team color (loads static obstacle map)
- `on` / `off` - Switch NeoPixel display mode

### Mission Planner

The strategy system uses a **block-based mission planner** with prioritized task scheduling:

```cpp
struct Block {
    const char* name;           // "collect_A"
    uint8_t priority;           // 0-255 (higher = first)
    uint16_t score;             // Points if SUCCESS
    uint32_t estimatedMs;       // Time budget
    BlockResult (*action)();    // Callable
    bool (*feasible)();         // Optional constraint
};
```

Selection modes:
- `PRIORITY` - Execute in descending priority order (deterministic).
- `SCORE` - Maximize score/time ratio among feasible blocks that fit in remaining time.

Safety margin (default 3000 ms) prevents blocks from starting too close to match end. Maximum 16 blocks per mission.

### Pin Mappings (T4.1)

| Pin | Function |
|-----|----------|
| 24 | Stepper enable |
| 25, 4, 5 | Stepper dir A, B, C |
| 27, 6, 26 | Stepper step A, B, C |
| 38 | Reset button |
| 32 | Starter pin |
| 31 | Team switch |
| 36 | Strategy switch |
| 2 | Twin switch |
| 28 | Traco power enable |
| 3 | Buzzer |
| 37 | NeoPixel data |

Actuators are driven via PCA9685 I2C PWM (servo pins 14, 15, 17, 20, 21, 22; pump/EV pins 0-3).

---

## Teensy 4.0 - Perception Module

### Overview

The T4.0 handles LIDAR data acquisition, obstacle detection, occupancy mapping, and visual feedback. It runs a simpler OS with three services: Lidar, Pixel (NeoPixel), and Intercom.

### Main Loop

```
setup()
  Console init (INFO level)
  Register BOOT + RUNNING routines

onBoot()
  Attach Lidar service
  Attach Pixel service
  Attach Intercom service
  Register intercom request callback

loop()
  os.run()
    -> updateServices() (sequential, all enabled services)
    -> process job queue
```

### LIDAR Processing (LD06)

The LD06 is a 360-degree scanning LIDAR connected via Serial1 at 230400 baud.

**Packet format**: 47 bytes per packet containing 12 data points. Each point has confidence (1 byte) and distance (2 bytes). Packets include start/end angles, speed, timestamp, and CRC.

**Processing pipeline**:
1. Read packet from Serial1 with CRC validation
2. Parse 12 points with interpolated angles between start/end angle
3. Convert polar (angle, distance) to cartesian (x, y) using robot position
4. Apply polar filter: distance 200-2500 mm, intensity >= 250
5. Apply cartesian filter: points must be within playing field (3000 x 2000 mm with 70 mm margin)
6. Store in polar grid (36 sectors of 10 degrees) and cartesian grid (20 x 13 cells)
7. Expire old points after 500 ms persistence

**Obstacle detection algorithm**: Multi-angle voting scheme checking 9 rays (center, +/-10, +/-20, +/-30, +/-40 degrees). Each ray has distance threshold (base + deceleration offset). A ray votes "obstacle" if at least 2 valid points are within threshold. Any positive ray triggers obstacle detection. Deceleration offsets: -20 mm at +/-20, -35 mm at +/-30, -55 mm at +/-40 degrees.

### Occupancy Map

Grid: 20 columns x 13 rows covering the 3000 x 2000 mm playing field (150 mm x ~154 mm per cell).

Compression: Bit-packed (260 cells -> 33 bytes), hex-encoded, with CRC-8 checksum. Format: `hexdata-crc`.

Content combines dynamic LIDAR detections with static per-team obstacle maps (loaded via `team(B|Y)` command).

### NeoPixel Ring (35 LEDs)

Three display modes:
- **INTERCOM**: Connection status - green pulsing (connected), red pulsing (disconnected)
- **LIDAR**: Distance visualization - each LED represents ~10.3 degrees. Color gradient from red (300 mm) to green (1000 mm).
- **TEAM COLOR**: Yellow or blue pulsing based on team selection.

Auto-switches from INTERCOM to LIDAR mode 5 seconds after connection.

### OLED Display (SH1106, 128x64)

Shows a scaled 20x13 occupancy grid with robot position marker. Updates every 500 ms. Connected via I2C (SDA=18, SCL=19).

### Pin Mappings (T4.0)

| Pin | Function |
|-----|----------|
| 4 | NeoPixel data (35 LEDs) |
| 6 | LIDAR LD06 speed PWM |
| 7 | LIDAR RX (Serial1 @ 230400) |
| 18 | OLED SDA (I2C) |
| 19 | OLED SCL (I2C) |
| Serial2 | Intercom (31250 baud) |

---

## Communication Protocol

### Frame Format

All framed messages use CRC8-SMBUS integrity checking:

```
<payload>|<crc>\n
```

Where `crc` is the decimal representation of `CRC8_SMBUS(payload_bytes)`.

### Message Types

| Type | Format | Direction |
|------|--------|-----------|
| Request | `uid:command|crc\n` | Sender -> Receiver |
| Reply | `ruid:response|crc\n` | Receiver -> Sender |
| Telemetry | `TEL:channel:data|crc\n` | Firmware -> Host |
| Plain message | `message\n` | Any (no CRC, no reply) |
| Ping | `ping\n` | Any |
| Pong | `pong\n` | Any |

### Transport Layers

| Link | Baud | Port | Between |
|------|------|------|---------|
| USB-CDC | 115200 | USB Serial | T4.1 <-> PC |
| Intercom | 31250 | Serial1/Serial2 | T4.1 <-> T4.0 |
| XBee | 31250 | /dev/ttyUSB0 | T4.1 <-> Jetson |

### Connection Handshake

USB-CDC: Host sends CRC-framed `hb` request. Any valid reply confirms connection.

XBee/Intercom: Sender sends `ping\n`, receiver replies `pong\n`.

---

## Match Timing Constants

| Constant | Value |
|----------|-------|
| Match duration | 100 s |
| Near-end warning | 10 s before end |
| End-match buffer | 200 ms |
| Mission safety margin | 3000 ms (default) |

---

## Console and Debug

The `Console` class provides leveled logging (VERBOSE, INFO, WARNING, CRITICAL, SUCCESS) with timestamp support. On T4.1, `CONSOLE_SERIAL` == `BRIDGE_SERIAL` when using USB, meaning Console output shares the same physical port as the bridge protocol. Unframed console lines (no `|crc` suffix) are forwarded to the holOS terminal via a dedicated `_console` subscriber channel.

Teleplot-compatible plotting: `Console::plot("name", "value")` outputs `>name:value` format. `Console::plotXY("name", "x", "y")` outputs `>name:x:y|xy`.
