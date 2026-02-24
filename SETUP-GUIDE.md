# RW-Template Setup Guide — What To Do Next

Your robot hardware is configured and code is written. Here's everything you need to do, in order.

---

## Step 1: Build & Upload

1. Open this project in VS Code with the **VEX Robotics Extension** installed
2. Click the **Build** button (or `Ctrl+Shift+B`)
3. Fix any build errors
4. Connect your V5 Brain via USB and click **Upload**

---

## Step 2: Verify Motor Directions

1. Run the robot in **driver control** mode
2. Push both joysticks forward — the robot should drive **forward**
3. If a side drives the wrong way, flip the `true`/`false` (reversed) flag for that motor in `custom/src/robot-config.cpp`

---

## Step 3: Verify Subsystems

Test each button in driver control:

| Button   | Action                                  |
| -------- | --------------------------------------- |
| **R1**   | Toggle intake forward                   |
| **A**    | Toggle intake reverse                   |
| **L1**   | Combo: intake forward + outtake reverse |
| **X**    | Toggle mid-scoring mode                 |
| **R2**   | Hold to retract Descore                 |
| **L2**   | Toggle Unloader                         |
| **DOWN** | Cycle drive curve type                  |

Also verify:

- Brain screen shows intake current (mA) and active curve name
- Controller shows curve name when cycling
- Motor temperature warnings appear at ≥50°C
- Low battery warning at ≤10%

---

## Step 4: Tune PID

**See `PID-TUNING-GUIDE.md` for the complete guide.**

Summary of what to tune in `custom/src/robot-config.cpp`:

| #   | Controller         | Variables                                   | Tune For                                       |
| --- | ------------------ | ------------------------------------------- | ---------------------------------------------- |
| 1   | Distance PID       | `distance_kp`, `distance_ki`, `distance_kd` | Straight-line driving accuracy                 |
| 2   | Turn PID           | `turn_kp`, `turn_ki`, `turn_kd`             | Turning accuracy at 30°, 90°, 180°, 270°, 360° |
| 3   | Heading Correction | `heading_correction_kp/ki/kd`               | Driving straight without curving               |

Set `auton_selected = 1` in `user.cpp` to run `tuningAuton()`.

---

## Step 5: Mount & Configure Distance Sensors

**See `DISTANCE-SENSOR-GUIDE.md` for the complete guide.**

### What You Need To Do:

- [ ] Mount 2 distance sensors on the **back** of the robot (spaced apart horizontally)
- [ ] Mount 1 distance sensor on the **left** side
- [ ] Mount 1 distance sensor on the **right** side
- [ ] Set sensor **ports** in `custom/src/robot-config.cpp` (lines 42-45)
- [ ] Measure and set sensor **offsets** in `custom/src/robot-config.cpp` (lines 99-103):

| Value                      | What to Measure                                                 |
| -------------------------- | --------------------------------------------------------------- |
| `back_sensor_left_offset`  | Distance from left back sensor to robot center (inches)         |
| `back_sensor_right_offset` | Distance from right back sensor to robot center (inches)        |
| `back_sensor_spacing`      | Center-to-center distance between the two back sensors (inches) |
| `left_sensor_offset`       | Distance from left sensor to robot center (inches)              |
| `right_sensor_offset`      | Distance from right sensor to robot center (inches)             |

---

## Step 6: Write Autonomous Routines

**See `AUTONOMOUS-GUIDE.md` for the complete guide.**

Edit `custom/src/autonomous.cpp` — placeholder functions are ready:

- `skills_auton()`
- `leftAuton()` / `leftAutonDescore()`
- `rightAuton()` / `rightAutonDescore()`

Select which runs by changing `auton_selected` in `custom/src/user.cpp`.

---

## Step 7: Tune Drive Curve

The robot starts with `CURVE_EXPONENTIAL`. Press **DOWN** during driver control to cycle through all 8 curves:

Linear → Exponential → Cubic → Quadratic → S-Curve → Squared → Piecewise → Plateau

Pick whichever feels best to your driver. You can change the default in `custom/src/robot-config.cpp` or `src/curves.cpp`.

---

## File Reference

Only edit files in the `custom/` folder — core files will be overwritten on template updates.

| File                            | Purpose                                         |
| ------------------------------- | ----------------------------------------------- |
| `custom/src/robot-config.cpp`   | Motor ports, sensors, PID values, measurements  |
| `custom/include/robot-config.h` | Device declarations (must match .cpp)           |
| `custom/src/user.cpp`           | Driver control, pre-auton init, auton selection |
| `custom/src/autonomous.cpp`     | Your autonomous routines                        |
| `custom/include/autonomous.h`   | Autonomous function declarations                |

---

## Detailed Guides

| Guide                      | What It Covers                                                      |
| -------------------------- | ------------------------------------------------------------------- |
| `PID-TUNING-GUIDE.md`      | Step-by-step PID tuning for all 3 controllers with validation tests |
| `AUTONOMOUS-GUIDE.md`      | Full motion function reference, code examples, chaining, tips       |
| `DISTANCE-SENSOR-GUIDE.md` | Sensor setup, measurements, reset functions, troubleshooting        |
