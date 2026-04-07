# holOS — Session Starter Prompt

Copy-paste this as the first message when starting a new Claude session on this project.

---

## Prompt

You are working on **holOS**, the full software stack for a holonomic competition robot (Coupe de France de Robotique). Read `AGENT.md` at the project root first — it is the authoritative reference for architecture, conventions, and constraints.

### Architecture in 30 seconds

```
Jetson / PC  ←— XBee or USB-CDC (framed CRC8 protocol) —→  Teensy 4.1 (motion, actuators, safety)
                                                                   │
                                                              UART intercom
                                                                   ↓
                                                              Teensy 4.0 (LIDAR, occupancy, LEDs)
```

- **Firmware** (`firmware/teensy41/`): PlatformIO C++. Service-oriented architecture. ISR-critical functions in ITCM, everything else `FLASHMEM`. Key services: Motion (stepper + PID), Safety (LIDAR), Actuators (servos + pumps), JetsonBridge (command dispatcher + fallback), Chrono (100s match timer).
- **Software** (`software/`): Python 3. `brain.py` orchestrates everything. Transport abstraction (`transport/`) lets the same strategy code run on real hardware or simulator. Services in `services/` mirror firmware capabilities. Strategy in `strategy/match.py` is hot-reloadable.
- **Entry point** (`software/run.py`): Unified for PC + Jetson. Flask + SocketIO on port 5000. Auto-detects platform (Linux=Jetson, Windows=PC). `sim/bridge.py` is a fake Teensy. `sim/physics.py` runs holonomic kinematics at 60Hz. `sim/world.py` has the occupancy grid + A* pathfinder.
- **Web UI** (`software/sim/static/`): Single-page app — `index.html`, `app.js`, `style.css`. Canvas-based field rendering, map pins, trajectory builder, remote control drawer, actuator page with sequence editor, opponent simulation, grid brush tools.
- **Protocol** (`shared/protocol.py`): `<id>:<cmd>|<crc8>\n` for commands, `r<id>:<response>|<crc8>\n` for replies, `TEL:<type>:<data>|<crc8>\n` for telemetry.

### Key files to read when starting

| Need to understand… | Read… |
|---|---|
| Full architecture + conventions | `AGENT.md` (root) |
| Wire protocol details | `ARCHITECTURE.md` (root) |
| Python config + constants | `software/shared/config.py` |
| Brain / orchestrator | `software/brain.py` |
| Unified entry point | `software/run.py` |
| Strategy (hot-reload) | `software/strategy/match.py` |
| Firmware services | `firmware/teensy41/src/services/` |
| Firmware commands | `firmware/teensy41/src/os/commands.cpp` |
| Web UI (JavaScript) | `software/sim/static/app.js` |

### How we work

- **Language**: French or English, I switch freely. Code comments and variable names stay in English.
- **My profile**: Advanced C++ developer (firmware + software), intermediate Python, occasional web (JS/React/HTML/CSS).
- **Workflow**: I describe what I want — you implement it. When touching firmware, be careful about `FLASHMEM` rules (see `AGENT.md`). When touching the web UI, keep all JS in `app.js`, all CSS in `style.css`, all HTML in `index.html` (no frameworks, no bundler).
- **Convention**: Math angles (0=East, CCW positive, North=90°). The firmware and UI share this convention. `turn(angle)` is absolute in table frame. `goPolar(relAngle, dist)` is relative to robot heading.
- **Testing**: Launch the sim with `cd software && python run.py --sim`, open `localhost:5000`. For firmware: `cd firmware/teensy41 && pio run` to compile, `pio run --target upload` to flash.
- **Don't**: Push to remote. Create commits without asking. Add README files I didn't request. Use frameworks or bundlers for the web UI. Skip reading `AGENT.md` when unsure about something.

### Current state (update this section each session)

<!-- Update these bullets before pasting the prompt -->
- Grid-cell brush for painting/erasing occupancy grid cells (replaces old dynamic round obstacles)
- Fake opponent with placement, sequence playback, and canvas rendering
- Actuator page: per-servo sliders, pose library (snapshot/apply/save/load), sequence builder (pose+delay+command steps, play/stop, export as Python code), pneumatics with `initPump()`
- Pathfinder: A* with obstacle inflation (ROBOT_RADIUS clearance) and diagonal corner-cut blocking
- Remote control: integrated as a slide-in drawer on the right side of the map view
- Left nav: SVG icons, collapsible groups (Game, Debug, Setup)
- Map: pin/popover system (Go, Set pos, Copy, Add waypoint), trajectory builder with copy/paste/execute

### What's next (update this too)

<!-- Fill in your current priorities -->
- Strategy development and mission testing
- Actuator sequence refinement for match actions
- Field testing with real hardware
