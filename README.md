# UMBC Micromouse Capstone

An autonomous micromouse developed for a 4×4 maze challenge. The robot senses nearby walls, builds a map as it moves, detects an opening at the maze boundary, and uses the discovered route to return to its start and make a faster second run.

This repository contains the embedded C++/Arduino firmware, maze-planning code, motion control, sensor integration, and earlier development experiments. The latest integrated firmware is in [`FINAL DEMO/`](FINAL%20DEMO/); the separately organized 4×4 planner and executor test sketch is in [`4x4_maze/`](4x4_maze/).

## Project at a glance

- **Platform:** Teensy 4.1 class embedded controller (firmware uses Arduino APIs)
- **Maze:** 4×4 physical maze represented as a 6×6 virtual grid, including an outer ring for exit cells
- **Sensing:** Three VL6180X time-of-flight range sensors, selected through a TCA9548A I²C multiplexer
- **Motion:** Differential drive with encoder feedback, motor driver, PID-based motion correction, and discrete cell/turn primitives
- **Planning:** Incremental wall mapping and floodfill distance propagation
- **Mission:** Explore to an exit, return to the start, orient, then run to the known exit

## System architecture

```text
VL6180X sensors ── I²C mux ──> Sensors ──> Perception ──> Planner / map
                                                           │
                                                           v
Encoders ───────────────────────────────────────────────> Executor
                                                           │
                                                           v
                                                Motion control ──> Motors
```

The main loop in `FINAL DEMO/MAY_8th.ino` coordinates sensor updates, mission state, planning, and execution. Sensor readings are converted into robot-relative wall observations. At cell stops, the planner records wall information and selects a direction. The executor translates that direction into a forward-cell or turn primitive, while motion control uses encoder counts and range readings to regulate movement.

## Hardware

The project hardware configuration documented for the robot is:

| Component | Role |
| --- | --- |
| Teensy 4.1 | Main embedded controller |
| 3× VL6180X ToF sensors | Left, front, and right distance measurements |
| TCA9548A I²C multiplexer | Selects one of the three same-address range sensors |
| TB6612FNG dual H-bridge | Drives the left and right motors |
| 2× N20 geared DC motors with encoders | Differential drive and wheel-motion feedback |
| 2S 7.4 V LiPo battery | Robot power |

The checked-in sensor driver uses mux address `0x70` and channels 1 (left), 0 (front), and 7 (right). The firmware defines motor-control and encoder pins in `motors.cpp` and `encoders.cpp`; confirm the wiring against the current physical robot before using those pin assignments as a build guide.

## Firmware modules

The current integrated implementation is under [`FINAL DEMO/`](FINAL%20DEMO/).

| File(s) | Responsibility |
| --- | --- |
| `MAY_8th.ino` | Setup, periodic sensor refresh, mission state machine, settling intervals, and completion handling |
| `sensors.cpp/.h` | Multiplexer selection and time-of-flight reads, packaged into a sensor frame |
| `perception.h` | Converts left/front/right ranges into wall/open observations using 90 mm thresholds |
| `robot_state.cpp/.h` | Current cell, heading, raw sensor frame, and perception frame |
| `maze.cpp/.h` | 6×6 virtual maze, 4×4 inner-cell boundaries, wall connectivity, and breadth-first flood distances |
| `planner_interface.cpp/.h` | Sensor-to-map updates, exit discovery, exploration direction, and floodfill route selection |
| `executor.cpp/.h` | Non-blocking movement state machine for cell advances, 90° turns, 180° turns, alignment, and wall approach behavior |
| `motion_control.cpp/.h` | PID-based forward wall correction and encoder-driven turn/motion control |
| `encoders.cpp/.h` | Interrupt-based wheel tick counters |
| `motors.cpp/.h` | Direction and PWM commands to the dual H-bridge |

The repository also retains [`4x4_maze/`](4x4_maze/) as a distinct planner/executor integration test. It uses a three-phase planner API: `PHASE_EXPLORE`, `PHASE_RETURN`, and `PHASE_SPEEDRUN`.

## Maze-solving behavior

### Latest integrated demo (`FINAL DEMO`)

1. **Explore to the exit.** The planner updates the current cell from valid sensor observations. Before the exit is known, it prefers an unvisited open neighbor, with a left/forward/right/back preference relative to the robot's heading.
2. **Detect the boundary opening.** An open observation on an inner-maze boundary can mark the adjacent virtual outer-ring cell as the exit. The final planner requires a valid range reading at least 80 mm to confirm this opening.
3. **Return to the start.** The planner computes floodfill distances from the start and chooses connected neighbors with decreasing distance.
4. **Orient and pause.** At the start, the mission controller aligns north and waits three seconds.
5. **Run to the known exit.** The planner recomputes distances from the discovered exit and follows the decreasing-distance route. It stops after the robot reaches a cell outside the effective 4×4 maze.

The maze representation stores cell connectivity as neighbor pointers. Floodfill performs a breadth-first traversal through open connections, so the planner's distance gradient reflects the currently mapped route. This is a shortest path in the known map; it does not establish a globally optimal route through unobserved passages.

### Standalone 4×4 planner test (`4x4_maze`)

The test planner represents the physical maze and surrounding exit ring in a 6×6 grid. It follows the left wall while exploring, records sensed walls, transitions to a floodfill route back to the start when it finds an exit, and then follows the floodfill gradient for a speed run. The test sketch schedules sensor reads every 20 ms and advances the executor every 10 ms.

## Testing and current limitations

`4x4_maze/test_Executor/test_Executor.ino` is an embedded integration-test sketch. It brings up the motors, encoders, planner, executor, and sensors; updates sensor/perception state; advances motion control; and emits serial diagnostics. The final demo also prints sensor values, mission modes, and a planner map to the serial console. These files show the intended test workflow, but the repository does not include a test report, simulation output, recorded maze run, or quantitative performance results.

**Sensor-driver test setting:** `FINAL DEMO/sensors.cpp` currently sets `FORCE_ALL_READINGS_VALID = true`. In this mode, read or multiplexer errors can be represented as a valid 255 mm “far” reading. That can make a sensor fault look like an open path. Treat this as a development/test configuration; disable it and verify error handling before relying on the firmware in an unattended run.

The repository does not include a top-level build configuration or dependency lockfile. The firmware references Arduino APIs and the Adafruit VL6180X library; board package and library versions should be recorded before reproducing a build. Several directories are prior iterations or experiments; the dated `FINAL DEMO` folder is the most complete integrated mission firmware in the current tree.

## Build and run

1. Open `FINAL DEMO/MAY_8th.ino` in the Arduino IDE or a compatible Teensy/Arduino build workflow.
2. Install the Teensy board support package and Adafruit VL6180X library, then select the matching board and port.
3. Verify sensor mux wiring, motor-driver wiring, encoder pins, battery, and wheel orientation against the physical robot.
4. Place the robot at the configured start cell `(1, 1)`, facing north, in the 4×4 maze.
5. Upload and open the serial monitor at **115200 baud**. Observe sensor, mission, and planner diagnostics during a controlled test.

The repository does not pin exact board-package or library versions, and the hardware-dependent firmware has not been independently compiled or run as part of this README update.

## Results

The checked-in code implements the complete explore → return → known-exit run sequence and includes serial diagnostics for observing mission state and the mapped maze. No measured completion time, success rate, repeatability data, or final demonstration recording is included in the repository, so no numerical performance claim is made here.

## My contribution

I was the software lead for the capstone and wrote approximately 80% of the robot code. My work covered the embedded planning and control software, integrating range sensing, maze mapping and floodfill planning, executor/motion behavior, and bringing the robot firmware together for testing and demonstration. I also assembled and soldered the latest robot, reviewed the PCB, tested the system, and prepared the showcase presentation.

The repository contains multiple iterations and test sketches; the contribution statement describes my overall project role, while the files here provide the implementation evidence available in this repository.

## Evidence that would strengthen this portfolio entry

- A short video showing the explore, return-to-start, and known-exit run on the physical 4×4 maze
- Clear photos of the assembled robot, sensor arrangement, PCB, and wiring
- A labeled system block diagram or photo annotated with the Teensy, mux, ToF sensors, motor driver, motors, and encoders
- A representative serial-log excerpt showing exit detection, phase/mode transitions, final pose, and mapped maze
- A test table with maze layouts, successful/failed trials, run time, and any repeatability or calibration results
- A wiring/pinout diagram and a bill of materials with confirmed part numbers
- A reproducible build note with Teensy board-package and Adafruit VL6180X library versions
- A concise test guide explaining how to run `4x4_maze/test_Executor/test_Executor.ino` and what behavior each test verifies

## Repository map

| Directory | Contents |
| --- | --- |
| `FINAL DEMO/` | Latest integrated mission firmware and core modules |
| `4x4_maze/` | 4×4 maze planner and executor integration-test code |
| `Archive/`, `new_maze_april2/`, `Fall_Demo/`, `Motion/`, `PreRTOSMotion/` | Earlier development iterations and experiments |
| `Simulation/`, `flood_sim/`, `floodfill_template/` | Maze/floodfill simulation and template code |

## License

This project is distributed under the [MIT License](LICENSE).
