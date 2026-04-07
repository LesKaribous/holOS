# AGENT.md — holOS Developer & AI Agent Guide

This document describes the project architecture, conventions, and development constraints for any human developer or AI agent working on this codebase.

---

## Project Overview

**holOS** is the software stack for a holonomic robot competing in the Coupe de France de Robotique. The system spans two hardware targets (Teensy microcontrollers), a Jetson edge computer, and a web-based control/simulation interface.

The robot is a three-wheeled holonomic platform (omnidirectional) with stepper motors, servos, pumps, LIDAR obstacle detection, and a camera.

---

## Repository Layout

```
holOS/
├── firmware/
│   ├── teensy41/          ← Main controller (C++, PlatformIO)
│   │   ├── platformio.ini
│   │   └── src/
│   │       ├── config/    ← env.h, settings.h, pin.h, poi.h, calibration.h
│   │       ├── os/        ← OS kernel: cycles, jobs, threads, commands
│   │       ├── program/   ← routines.cpp (boot/manual/auto), strategy.cpp, match.cpp
│   │       ├── services/  ← All hardware services (motion, safety, actuators…)
│   │       └── utils/     ← geometry, interpreter, timer, planner
│   └── teensy40/          ← Perception co-processor (LIDAR, occupancy, NeoPixel)
│
├── software/
│   ├── run.py             ← Unified entry point (PC + Jetson, sim + hardware)
│   ├── run_hardware_tests.py ← Hardware test runner
│   ├── brain.py           ← Jetson orchestrator
│   ├── transport/         ← Communication layer (abstract + XBee + wired + virtual)
│   ├── services/          ← Python services (motion, safety, chrono, vision, occupancy)
│   ├── shared/            ← Protocol encoding, shared config constants
│   ├── sim/               ← Simulator physics engine + SimBridge (fake Teensy)
│   │   └── static/        ← holOS web UI: index.html, app.js, style.css
│   ├── strategy/          ← match.py (hot-reloadable strategy) + missions.json + macros.json
│   ├── tests/             ← Hardware test definitions
│   └── vision/            ← Camera integration (separate venv)
│
├── docs/                  ← Field maps, schematics, cheat sheets
├── deploy/                ← Jetson setup / update scripts
├── ARCHITECTURE.md        ← XBee protocol and high-level design
└── firmware/FIRMWARE_DOCUMENTATION.md ← Teensy services reference
```

---

## Hardware Topology

```
                  XBee 868 MHz or USB-CDC (57600 baud default)
 Jetson / PC  <─────────────────────────────────>  Teensy 4.1  (Main Controller)
 (holOS)           framed protocol, CRC8-SMBUS          │
                                                         │ UART 31250 baud (Intercom)
                                                         ▼
                                                    Teensy 4.0  (Perception)
                                                    LIDAR LD06, Occupancy map,
                                                    NeoPixel ring, OLED display
```

The Teensy 4.1 is always the authority for motion, safety, and actuation. The Jetson sends high-level commands and reads telemetry. Two strategy modes exist: **Remote** (strat=1, Jetson runs match.py) and **Internal** (strat=0, Teensy runs C++ blocks autonomously). In Remote mode, if the Jetson disconnects, the Teensy executes a fallback strategy.

---

## Firmware (Teensy 4.1)

### Build System

PlatformIO, target `teensy41`. Build from `firmware/teensy41/`:

```bash
pio run                  # compile
pio run --target upload  # flash via USB
pio device monitor       # serial console at 115200 baud
```

Key build flags (in `platformio.ini`):
- `-D TEENSY41` — hardware target guard
- `-D USB_DIRECT` — enables USB-CDC bridge path for wired holOS connection
- `-Os -ffunction-sections -fdata-sections -Wl,--gc-sections` — size optimization with dead-code elimination

Libraries:
- `TwinsyStep` (fork of TeensyStep) — stepper motor control
- `TeensyThreads` — preemptive threading for ISR cycles
- `SparkFun QWIIC OTOS` — optical odometry sensor
- `Adafruit PWM Servo Driver` — PCA9685 for servos/pumps
- `FastCRC` — CRC8-SMBUS hardware acceleration

### Memory Constraints — CRITICAL

The Teensy 4.1 has **two code memory regions**:

| Region | Size    | Location    | Access speed |
|--------|---------|-------------|--------------|
| ITCM   | ~512 KB | RAM1 (fast) | Zero-wait    |
| Flash  | ~7.8 MB | Flash (XIP) | ~1 cycle extra |

By default, all compiled code lands in **ITCM (RAM1)**. With a large codebase this overflows. To move functions to flash (freeing ITCM), use:

```cpp
FLASHMEM void MyClass::myFunction() { ... }
```

**Rule: apply `FLASHMEM` to every function that is NOT called from an ISR context.** Functions that MUST stay in ITCM (never add FLASHMEM):

- `motion.step()` — stepper ISR, runs every 10 µs
- `motion.control()` — PID loop, runs every 1 ms
- `positionController::step()`, `::control()`, `::onUpdate()`
- `stepperController::step()`, `::control()`, `::onUpdate()`
- `velocityController::step()`, `::control()`
- Any callback registered with `CycleManager` as a timed cycle

If you add FLASHMEM to an ISR-called function, the CPU will fault (data bus error) because flash is not accessible from ISR context on Cortex-M7 without MPU reconfiguration.

**When adding new services or large files:** if the build reports `RAM1: free for local variables: negative`, apply `FLASHMEM` to all non-ISR member functions in the new file.

### Service Architecture

Every hardware subsystem is a `Service` subclass registered with the OS:

```cpp
class MyService : public Service {
public:
    FLASHMEM void attach() override;  // called once on registration
    FLASHMEM void run()   override;  // called every main loop iteration (~2 ms)
};
extern MyService myService;          // singleton defined in env.h or service .cpp
```

Registration order in `onRobotBoot()` (routines.cpp) matters because services may depend on each other. The established order is:

1. IHM → Actuators → Motion → Chrono → Safety
2. SDCard (+ calibration load + mission load)
3. Motion engage → Localisation calibrate → Motion disengage
4. Intercom → JetsonBridge (+ fallback registration)
5. Lidar → Terminal → Vision
6. registerCommands()

To add a new service:
1. Create `src/services/myservice/myservice.h/.cpp`
2. Add the `ServiceID` enum value in `service.h` (before `ID_NOT_A_SERVICE`)
3. Add the case in `service.cpp` (`toID` / `toString`)
4. Include the header in `src/config/env.h`
5. Call `os.attachService(&myservice)` in `onRobotBoot()`

### OS State Machine

```
BOOT → MANUAL_PROGRAM ↔ MANUAL → AUTO_PROGRAM → AUTO → STOPPED
```

- **BOOT**: hardware init, service registration, calibration load
- **MANUAL_PROGRAM**: pre-match setup (runs `programManual()`)
- **MANUAL**: preparation phase — team selection, recalage, terminal commands, IHM interaction
- **AUTO_PROGRAM**: transition to match (runs `programAuto()`, arms robot via `robotArmed()`)
- **AUTO**: match running — if strat=1 (Remote), Jetson sends commands; if strat=0 (Internal), C++ BlockRegistry runs
- **STOPPED**: match ended — `onMatchEnd()` disables services, only `ihm.run()` stays active. Serial becomes unresponsive (no `updateServices()` in stop routine)
- Transition to AUTO: starter inserted (arms) → removed (starts); or `match_start` command from webapp; or `start` terminal command
- `onMatchEnd()` has a double-call guard (static bool). Calls `motion.disengage()` BEFORE `motion.disable()` (order matters due to `SERVICE_METHOD_HEADER`)

### Timing Cycles

| Period  | Function           | Must stay in ITCM |
|---------|--------------------|-------------------|
| 10 µs   | `motion.step()`    | YES               |
| 1 ms    | `motion.control()` | YES               |
| ~2 ms   | `os.run()`         | No (main loop)    |

Use `RUN_EVERY(code, interval_ms)` macro inside `run()` or routine callbacks for periodic tasks without blocking.

### Commands System

Terminal commands are registered in `os/commands.cpp` via `registerCommand("name(args)", handler_fn)`. Each command handler signature:

```cpp
FLASHMEM void command_mycommand(Request& req) {
    // req.getArg(0) — first argument as String
    req.reply("ok");   // or req.replyError("msg")
}
```

Commands can be invoked from the USB serial terminal, from the Jetson over XBee, or from the holOS web UI. Notable command groups:
- Motion: `go`, `goPolar`, `turn`, `align`, `goAlign`, `go_coc` (cancel-on-stall), `via`, `cancel`, `pause`, `resume`, `feed`
- Match control: `start`, `stop`, `match_start`, `match_stop`, `match_resume`
- Block registry: `blocks_list`, `run_block(name)`, `block_done(name)`
- Mission SD: `mission_sd_open`, `mission_sd_line(text)`, `mission_sd_close`, `mission_run`, `mission_abort`
- RuntimeConfig: `cfg_list`, `cfg_set(key,value)`, `cfg_save`
- Bridge: `hb`, `health`, `tel`, `occ`, `fb(id)`

### JetsonBridge & Fallbacks

Fallback IDs (enum `FallbackID`):

| Value | ID               | Default behavior         |
|-------|------------------|--------------------------|
| 0     | `STOP`           | cancel + disengage       |
| 1     | `WAIT_IN_PLACE`  | cancel (stay engaged)    |
| 2     | `RETURN_TO_BASE` | go to POI::home          |
| 3     | `CUSTOM_1`       | SD mission strategy      |
| 4     | `CUSTOM_2`       | (free)                   |

The heartbeat timeout (5 s without `hb` command) triggers `CUSTOM_1`, which runs the SD mission strategy if one is loaded, otherwise stops the robot. **The fallback only fires in Remote mode** (`os.getState() == AUTO && ihm.strategySwitch.getState() == true`). In Internal mode (strat=0), the C++ already handles the match. Register fallbacks in `onRobotBoot()` after `jetsonBridge.enable()`.

Python heartbeat tolerance: the transport now tolerates **3 consecutive heartbeat failures** before disconnecting (was 1). This avoids spurious disconnections during busy serial (heavy Console output during strategy execution).

### MissionController (SD Fallback)

Located at `src/services/mission/mission_controller.h/.cpp`.

- `MissionController::load()` — reads `/mission_fallback.cfg` from SD; called at boot
- `MissionController::execute()` — blocking execution of all loaded commands
- `MissionController::abort()` — stops execution mid-sequence
- `MissionController::sdOpen/AppendLine/Close()` — write a new strategy to SD (called by `mission_sd_*` commands); `sdClose()` calls `load()` to reload immediately

The `.cfg` format is one firmware command per line, human-readable:
```
# Approche zone A
go(1000,500)
turn(90)
raise(AB)
grab(AB)
delay(500)
go(0,0)
```

Comments (`#`) and blank lines are ignored. Motion commands (`go`, `goPolar`, `turn`, `align`, `goAlign`) are executed with blocking polling (`motion.hasFinished()`, timeout 30 s). `delay(ms)` is handled natively. All other commands are dispatched through `CommandHandler::execute()`.

---

## Software (Python / holOS)

### Running the Simulator

```bash
cd software/
pip install -r requirements.txt
python run.py --sim
# Open http://localhost:5000
```

The simulator provides a full virtual Teensy via `SimBridge`. The strategy (`strategy/match.py`) is hot-reloaded on every file save. Physics runs at 60 Hz with A* pathfinding, collision detection, and safety lookahead.

### Running on Real Hardware

```bash
# PC with USB connection:
python run.py --connect COM6

# Jetson (auto-detects Linux → connects /dev/ttyUSB0):
python run.py

# Jetson with specific port:
python run.py --connect /dev/ttyTHS1
```

Common XBee/USB ports on Jetson:
- `/dev/ttyUSB0` — USB XBee dongle
- `/dev/ttyTHS1` — Jetson Nano UART header
- `/dev/ttyTHS0` — Jetson Orin UART header

### Transport Layer

All communication goes through an abstract `Transport` base class (`transport/base.py`). Concrete implementations:

| Class            | File             | Usage                              |
|------------------|------------------|------------------------------------|
| `XBeeTransport`  | `xbee.py`        | XBee radio (57600 baud default)    |
| `WiredTransport` | `wired.py`       | USB-CDC direct (thin alias for XBeeTransport) |
| `VirtualTransport` | `virtual.py`   | Simulator (in-process SimBridge)   |

**Thread-safety constraints on transport:**
- `serial.write()` is NOT thread-safe in pyserial. All writes must go through `_write_lock` (threading.Lock already in place in `xbee.py`).
- The heartbeat thread skips its heartbeat when `_waiting_motion=True` (motion command in progress) to avoid frame interleaving.
- On heartbeat failure, call `self.disconnect()` (not `self._connected = False`) to ensure `_cleanup()` closes the serial port and allows reconnection.

**Motion completion (ack_done):**
The firmware retransmits `DONE:` telemetry every 2 s until `ack_done` is received. The transport's `execute()` for motion commands handles this internally. For `fire()` flows (web UI), an automatic `ack_done` is sent in the `_on_motion` telemetry callback when `_waiting_motion` is False.

### Communication Protocol

Frame format (Jetson ↔ Teensy):

```
Jetson → Teensy:  <id>:<cmd>|<crc8>\n       e.g.  42:go(500,300)|123\n
Teensy → Jetson:  r<id>:<response>|<crc8>\n  e.g.  r42:ok|45\n
Telemetry (push): TEL:<type>:<data>|<crc8>\n e.g.  TEL:pos:x=500,y=300,theta=1.57|78\n
Heartbeat:        hb / pong\n
```

CRC: **CRC8-SMBUS** (polynomial 0x07, init 0x00) — Python: `crcmod`, C++: `FastCRC`.

Telemetry channels (compact format, configurable rates via RuntimeConfig):

| Channel  | Alias  | Rate (default) | Compact format                              |
|----------|--------|----------------|---------------------------------------------|
| `T:p`    | `pos`  | 10 Hz (100ms)  | `x_mm y_mm theta_mrad` (space-separated ints) |
| `T:m`    | `motion`| 10 Hz + event | `R tx ty dist feed%` (running) / `I feed%` (idle) / `DONE:ok\|fail` |
| `T:s`    | `safety`| 2 Hz + change | `0` or `1`                                  |
| `T:c`    | `chrono`| 1 Hz (1000ms) | `<elapsed_ms>`                              |
| `T:od`   | `occ`  | 2 Hz (500ms)   | compressed hex occupancy map                |
| `T:mask` | `mask` | on-change      | `11110` (flags: pos,motion,safety,chrono,occ) |

Rates configurable on SD: `tel.pos_ms=100`, `tel.motion_ms=100`, etc. Legacy `TEL:` prefix is still parsed by Python for backwards compatibility.

Python subscribes to both compact and legacy names: `t.subscribe_telemetry('p', _on_pos)` + `t.subscribe_telemetry('pos', _on_pos)`.

### Strategy System

The strategy file `software/strategy/match.py` is **hot-reloadable**: saving the file during a simulator run reloads it automatically.

```python
def run_mission():
    safety.enable()
    m = Mission(chrono, log)
    m.set_mode(Mission.PRIORITY)  # or Mission.SCORE
    m.add(Block("collect_A", priority=10, score=80, time_ms=7000,
                action=block_collect_A,
                feasible=lambda: not occupancy.is_zone_occupied(POI.stock_1, 450)))
    m.run()
```

The same strategy code runs identically on the simulator and the real robot (services API is identical).

### Missions & Macros (holOS Mission Control)

- `strategy/missions.json` — list of mission definitions (approach point, macro reference, score, priority, enabled)
- `strategy/macros.json` — list of macro definitions (sequences of typed steps)

Mission JSON schema:
```json
{
  "id": "uuid",
  "name": "Collect Zone A",
  "desc": "Optional description",
  "approach": { "x": 1000, "y": 500, "angle": 90 },
  "macro_id": "macro-uuid",
  "score": 5,
  "priority": 10,
  "time_ms": 8000,
  "enabled": true
}
```

Macro step types: `move_to`, `face`, `actuator`, `wait`, `call_macro`, `if_occupied`, `log`.

**SD Fallback deploy flow:**
1. holOS calls `POST /api/missions/deploy-sd` → Python generates `fallback.cfg` from missions + macros
2. Python sends `mission_sd_open` → lines via `mission_sd_line(...)` → `mission_sd_close`
3. Firmware writes the file to SD and immediately reloads it into MissionController

### holOS Web UI

Single-page app at `software/sim/static/`. Views: Map, Strategy, Missions, Macros, Tests, Calibration, Settings.

Key API endpoints (Flask, `run.py`):

| Method | Path                             | Description                          |
|--------|----------------------------------|--------------------------------------|
| GET    | `/api/missions`                  | Load missions.json                   |
| PUT    | `/api/missions`                  | Save missions.json                   |
| POST   | `/api/missions/deploy-sd`        | Generate + deploy fallback.cfg to SD |
| POST   | `/api/missions/preview-fallback` | Generate fallback.cfg (no write)     |
| POST   | `/api/missions/run-step`         | Execute a single macro step on robot |
| GET    | `/api/macros`                    | Load macros.json                     |
| PUT    | `/api/macros`                    | Save macros.json                     |
| POST   | `/api/execute`                   | Execute a command on robot           |

SocketIO events: `state` (full robot state at 10Hz), `motion_done`, `serial_status`, `match_state`, `cpp_block_result`, `test_progress`, `test_result`, `test_done`, `reload`.

---

## Reliability Constraints

These are non-negotiable hard constraints tied to competition rules and hardware safety.

### Match Timer — 100 seconds

The match runs for exactly 100 s (`Settings::Match::DURATION = 100000 ms`). Any code path that runs during `AUTO_PROGRAM` must respect this budget. Key timing guards:

- `Settings::Match::NEARLY_FINISH = 10000 ms` — the `onMatchNearEnd()` callback fires at 10 s remaining. Raise actuators, return to safe zone, stop scoring actions here.
- `Settings::Match::ENDMATCH = 200 ms` — the `onMatchEnd()` callback fires at T−200 ms. Everything must be stopped within this window. Motors WILL be cut by the organizers' power strip at T=0; if the robot is still moving it counts as a penalty.
- In `match.py` (Python strategy), the `Mission` planner already respects `chrono.getTimeLeft()` with a safety margin. Do not remove this guard.
- In `MissionController::execute()` (SD fallback), each motion command has a 30 s hard timeout. This prevents an infinite block if the robot stalls, but you must still design macros that fit in the remaining match time.

### Fallback Chain

When the Jetson disconnects mid-match **in Remote mode (strat=1)**, the firmware must react within the heartbeat timeout (default 5 s, `m_heartbeatTimeoutMs`). The chain is:

```
Jetson silent > 5s  AND  os.getState() == AUTO  AND  strat=1 (Remote)
    → JetsonBridge::_checkWatchdog() fires
    → triggerFallback(FallbackID::CUSTOM_1)
        → if SD strategy loaded: MissionController::execute()  (blocking, motor-safe)
        → else: motion.cancel() + disengage  (safe stop)
```

**In Internal mode (strat=0), the fallback watchdog is disabled** — the C++ already runs the match autonomously.

Fallback IDs are registered in `onRobotBoot()` and cannot be changed at runtime. If you need a new autonomous behavior on disconnect, use `CUSTOM_2` (currently free) or modify the `CUSTOM_1` lambda.

**Never** let the fallback path call any function that requires Jetson connectivity (XBee send, `intercom.sendRequest`, etc.). The Jetson is gone by definition.

### Safety System

The Safety service runs on every `os.run()` iteration and reads obstacle data from the LIDAR (via Intercom from T4.0). It is:
- Disabled in `MANUAL_PROGRAM` (pre-match preparation)
- Enabled at match start via `safety.enable()` in `programAuto()` and in all fallback lambdas that perform motion

**Safety must be enabled before any autonomous motion** during a real match. Not enabling it means the robot can collide with the opponent without braking.

Safety is intentionally **never delegated to the Jetson** — it runs locally on T4.1 at all times. The Jetson cannot override an active safety stop. This is a deliberate design choice.

### Motion Commands and Timing

- Motion commands (`go`, `goPolar`, `turn`, `align`, `goAlign`) are **blocking by default** from the Jetson/Python side: the transport waits for `DONE:ok` or `DONE:fail` before returning.
- From the firmware side (`routines.cpp`, fallback lambdas), motion calls are blocking unless prefixed with `async`.
- After a `DONE:` event, the firmware retransmits every 2 s until `ack_done` is received. Always call or auto-ack after motion completes (this is handled by the transport layer for `execute()` flows and by auto-ack in `_on_motion` for `fire()` flows).
- The `MissionController` polls `motion.hasFinished()` with a 30 s timeout. If a motion stall is not caught before this timeout, the next command executes anyway — design macros with stall guards.

### StallDetector

The motion system includes a stall detector (`stallDetector.h/.cpp`) that checks if the robot has made progress on a sliding window. It can be controlled per-move:

```cpp
async motion.noStall().go(x, y);        // disable for this move
async motion.withStall(false).go(x, y); // same
```

Default thresholds are in `Settings::Motion::Stall`. Calibrate by calling `motion.printDiagReport()` after a move — the `stallMinTransMm` field shows the minimum displacement observed, set `TRANS_DISP_MM` slightly below that value.

To disable globally: set `stallEnabled = false` in `MoveOptions` struct (motion.h).

### SD Card & RuntimeConfig

The SD card (Teensy 4.1 built-in) is used for:
- Calibration save/load (`/calibration.json`)
- Mission fallback strategy (`/mission_fallback.cfg`)
- Runtime configuration (`/config.cfg`) — key=value pairs loaded at boot

RuntimeConfig (`runtime_config.h/.cpp`) provides a lightweight key=value store:
- `RuntimeConfig::load()` — reads `/config.cfg` at boot
- `RuntimeConfig::getInt("tel.pos_ms", 100)` — typed getters with defaults
- `RuntimeConfig::set("key", "value")` + `RuntimeConfig::save()` — modify + persist
- Terminal: `cfg_list`, `cfg_set(key,value)`, `cfg_save`

Currently used for: telemetry rates (`tel.pos_ms`, `tel.motion_ms`, etc.), actuator servo limits.

SD access is SPI-based and can take 5–50 ms per operation. **Never call SD functions from ISR context or from within the stepper/PID cycles.** All SD access happens in the main OS loop (boot, command handlers, `MissionController::sdClose()`).

---

## Log & Telemetry Configuration

### The Problem Without holOS

When connecting via a plain USB serial terminal (PuTTY, `pio device monitor`, screen…), the T4.1 outputs:
- Console messages from all services (some high-frequency)
- JetsonBridge telemetry frames at 10 Hz for pos/motion/safety/chrono + 2 Hz for occupancy

This floods the terminal and makes strategy-level logs unreadable. The solution is to configure the log state at boot and adjust it at runtime.

### Boot Defaults (`settings.h`)

```cpp
namespace Settings {
  namespace Log {
    constexpr ConsoleLevel BOOT_LEVEL = ConsoleLevel::INFO;

    // Per-service console output at boot (false = muted)
    constexpr bool SRC_INTERCOM     = false;  // protocol-level noise
    constexpr bool SRC_LOCALISATION = false;  // high-frequency OTOS updates
    constexpr bool SRC_NEOPIXEL     = false;  // LED driver noise
    // all others default to true

    namespace Telemetry {
      constexpr bool POS    = true;
      constexpr bool MOTION = true;
      constexpr bool SAFETY = true;
      constexpr bool CHRONO = true;
      constexpr bool OCC    = true;
    }
  }
}
```

Change any of these to `false` to mute that source from power-on. The settings take effect in `Console::init()` (called from `setup()` before `os.run()`).

### Runtime Commands (Serial Terminal)

These commands work over USB serial without holOS:

```
logstatus              → print current level + all source states
loglevel(VERBOSE)      → show all messages including trace
loglevel(WARNING)      → only warnings and errors
loglevel(DISABLED)     → silence everything

log(MOTION, 0)         → mute Motion service logs
log(INTERCOM, 1)       → re-enable Intercom logs
log(LOCALISATION, 0)   → mute high-frequency OTOS
log(*, 1)              → enable all sources
log(*, 0)              → mute all service sources (string-origin logs still show)

tel(occ, 0)            → stop occupancy map telemetry frames
tel(chrono, 0)         → stop chrono telemetry
tel(pos, 0)            → stop position telemetry
tel(*, 0)              → silence all telemetry (pure debug mode)
tel(*, 1)              → restore all telemetry
```

### Typical Workflow: Debugging Strategy Without holOS

1. Connect USB serial terminal at 115200 baud
2. Boot — you'll see the ASCII banner + boot progress
3. Optionally quiet the noise: `tel(*,0)` then `log(*,0)` then `log(TERMINAL,1)`
4. Send terminal commands to test motion/actuators/strategy
5. Check strategy output with `loglevel(VERBOSE)` + `log(MOTION,1)` for detailed diagnostics
6. Restore telemetry before reconnecting holOS: `tel(*,1)`

### Console Architecture (for contributors)

```
Console::info(ID_MOTION)   → checks m_sourceMask bit for ID_MOTION
                           → if filtered: ConsoleStream(INFO, "MOTION", forceIgnore=true)
                           → if allowed:  ConsoleStream(INFO, "MOTION", false)
                                          → prints [Info](MOTION): ...

Console::info("Fallback")  → no source-mask check (String origin)
                           → ConsoleStream(INFO, "Fallback", false)
                           → prints [Info](Fallback): ...
                           → filtered only by level, not by source mask
```

String-origin messages (from `routines.cpp`, fallback lambdas, strategy code) are **never filtered by the source mask**. They represent program-level events that you always want to see. Only service-class messages (`Console::info(ID_xxx)`) respect the per-source filter.

---

## Development Constraints & Pitfalls

### Firmware

1. **Never add `FLASHMEM` to ISR-called functions** (`step`, `control`, `onUpdate` in any motion controller). The CPU will fault at runtime with no diagnostic output.

2. **Monitor RAM1 usage** after every significant addition. Build output shows: `RAM1: variables:..., code:..., free for local variables:...`. If free goes negative, apply FLASHMEM to the new file's non-ISR functions.

3. **The `async` keyword** before motion calls (`async motion.go(...)`) is a macro that makes the call non-blocking. Without it, motion calls block until completion. In ISR callbacks and fallback lambdas, always use `async` unless you intentionally want blocking.

4. **SD card access** must not happen during motion steps or ISR cycles. SD reads/writes are slow (SPI-based) and will cause step timing jitter. `MissionController::load()` is only called at boot and after `mission_sd_close`; `execute()` is only called in fallback context when the Jetson is already disconnected.

5. **Thread safety in C++**: TeensyThreads are preemptive. If a service's `run()` shares state with a cycle callback, protect with a flag or atomic. Motion's internal state is protected by the existing cycle architecture.

6. **String handling**: Teensy uses Arduino `String` class (heap-allocated). In tight ISR loops (10 µs step), never allocate `String` objects — use `const char*` or fixed char arrays.

7. **PlatformIO environment**: always build for `teensy41` env. The `teensy35` envs are legacy and unused.

### Python / Software

1. **pyserial is not thread-safe**: Never call `serial.write()` from two threads simultaneously. Always use `_write_lock` in transport implementations.

2. **Heartbeat vs. motion race**: The heartbeat thread must skip its `hb` command while a motion command is in progress (`_waiting_motion=True`). Concurrent writes during motion will corrupt the framing.

3. **Disconnect/reconnect**: Use `transport.disconnect()` (not `transport._connected = False`) on error. Only `disconnect()` calls `_cleanup()` which closes the serial port; otherwise the port stays open and `WiredTransport.__init__()` fails to reopen it.

4. **`fire()` vs `execute()`**: `execute()` is request-response (framed, CRC, waits for reply). `fire()` is best-effort (unframed, no reply wait). Use `execute()` for commands that need confirmation. Use `fire()` only for telemetry/status queries where loss is acceptable.

5. **ack_done for DONE telemetry**: After a motion command completes, the firmware retransmits `DONE:` every 2 s until `ack_done` is received. The `execute()` path handles this. The `fire()` / web UI path relies on the auto-ack mechanism in `_on_motion` callback.

6. **Strategy hot-reload**: `match.py` is reloaded via `importlib`. Do not use module-level mutable state that persists across reloads — reinitialize everything inside `run_mission()`.

### holOS UI

1. **View `calibration`** must stay inside `<div id="views">` in `index.html`. It was historically placed outside (inside the color popup) and became invisible.

2. **Map canvas click handler**: currently used for approach point picking in Missions view (`_missionPickingApproach` flag). Any new click handler must be coordinated with this flag.

3. **Macro steps in macros.json**: steps of type `if_occupied` and `log` are skipped when generating `fallback.cfg` (firmware doesn't support them). Steps of type `call_macro` are inlined recursively. Keep this in mind when designing macros intended for SD fallback.

---

## Key Files Quick Reference

| File | Role |
|------|------|
| `firmware/teensy41/src/config/env.h` | Global includes, all service singletons |
| `firmware/teensy41/src/config/settings.h` | All tunable constants (PID, speeds, baudrates) |
| `firmware/teensy41/src/config/poi.h` | Points of interest (named coordinates) |
| `firmware/teensy41/src/program/routines.cpp` | Boot sequence, manual/auto loops, fallback registration |
| `firmware/teensy41/src/program/strategy.cpp` | Embedded fallback strategy (C++) |
| `firmware/teensy41/src/os/commands.cpp` | All terminal/remote command registrations |
| `firmware/teensy41/src/services/jetson/jetson_bridge.cpp` | XBee/USB command dispatch, telemetry, watchdog |
| `firmware/teensy41/src/services/mission/mission_controller.cpp` | SD strategy loader/executor |
| `firmware/teensy41/src/services/motion/motion.cpp` | Holonomic motion control |
| `software/run.py` | Flask server + all Python API endpoints |
| `software/transport/xbee.py` | XBee transport (thread-safe write, heartbeat) |
| `software/strategy/match.py` | Hot-reloadable match strategy |
| `software/strategy/missions.json` | Mission definitions |
| `software/strategy/macros.json` | Macro step sequences |
| `software/sim/static/index.html` | holOS web UI structure |
| `software/sim/static/app.js` | holOS web UI logic |
| `software/sim/static/style.css` | holOS web UI styles |
