# Autonomous Routes Guide — RW-Template

This guide covers how to write autonomous routines using the RW-Template motion functions.

---

## File Structure

| File                          | Role                                         |
| ----------------------------- | -------------------------------------------- |
| `custom/src/autonomous.cpp`   | Where you write your autonomous routines     |
| `custom/include/autonomous.h` | Declare new function names here              |
| `custom/src/user.cpp`         | Select which routine runs (`auton_selected`) |

---

## Quick Start: Your First Autonomous

### 1. Set the Starting Position

At the top of your routine, set where the robot starts on the field:

```cpp
void leftAuton() {
  // Robot starts at (-48, -60) facing 0° (toward positive Y)
  x_pos = -48;
  y_pos = -60;
  correct_angle = 0;

  // Your moves go here...
}
```

> **Coordinate system:** (0, 0) is the center of the field. X is left/right, Y is forward/back. 0° = facing the positive Y direction. Angles increase clockwise.

### 2. Write Your Moves

```cpp
void leftAuton() {
  x_pos = -48; y_pos = -60; correct_angle = 0;

  // Drive forward 24 inches
  driveTo(24, 3000);

  // Turn to face 90 degrees
  turnToAngle(90, 2000);

  // Score with intake
  intake_motor.spin(fwd, 12, volt);
  wait(1000, msec);
  intake_motor.stop(hold);

  // Drive backward
  driveTo(-12, 2000);
}
```

### 3. Select It in user.cpp

In `custom/src/user.cpp`, change the auton selector:

```cpp
int auton_selected = 3;  // 3 = leftAuton
```

Selector map:
| Value | Routine |
|---|---|
| 1 | `tuningAuton()` |
| 2 | `skills_auton()` |
| 3 | `leftAuton()` |
| 4 | `rightAuton()` |
| 5 | `rightAutonDescore()` |
| 6 | `leftAutonDescore()` |

---

## Motion Function Reference

### `driveTo(distance, timeout, exit, max_output)`

Drive straight forward or backward.

| Parameter    | Type   | Default | Description                                                            |
| ------------ | ------ | ------- | ---------------------------------------------------------------------- |
| `distance`   | double | —       | Target distance in **inches**. Positive = forward, negative = backward |
| `timeout`    | double | —       | Max time in **milliseconds**                                           |
| `exit`       | bool   | `true`  | `true` = stop at end, `false` = keep moving (for chaining)             |
| `max_output` | double | `12`    | Max motor voltage (0-12). Lower = slower but more controlled           |

```cpp
driveTo(24, 3000);              // 24 inches forward, 3 second timeout
driveTo(-18, 2000);             // 18 inches backward
driveTo(36, 3000, true, 8);     // 36 inches at ~67% speed
```

---

### `turnToAngle(angle, timeout, exit, max_output)`

Turn to an **absolute** heading (not relative).

| Parameter    | Type   | Default | Description                                            |
| ------------ | ------ | ------- | ------------------------------------------------------ |
| `angle`      | double | —       | Target heading in **degrees** (absolute, not relative) |
| `timeout`    | double | —       | Max time in ms                                         |
| `exit`       | bool   | `true`  | `true` = stop at end                                   |
| `max_output` | double | `12`    | Max motor voltage                                      |

```cpp
turnToAngle(90, 2000);          // Face 90 degrees
turnToAngle(-45, 2000);         // Face -45 degrees
turnToAngle(0, 2000);           // Face back to 0
```

> **Important:** Angles are **absolute**, not relative. If you're at 90° and call `turnToAngle(180, 2000)`, the robot turns 90° more — it doesn't turn 180°.

---

### `moveToPoint(x, y, dir, timeout, exit, max_output, overturn)`

Drive to a specific field coordinate. The robot will adjust its heading while driving.

| Parameter    | Type   | Default | Description                                         |
| ------------ | ------ | ------- | --------------------------------------------------- |
| `x`, `y`     | double | —       | Target coordinates in **inches**                    |
| `dir`        | int    | —       | `1` = drive forward to point, `-1` = drive backward |
| `timeout`    | double | —       | Max time in ms                                      |
| `exit`       | bool   | `true`  | `true` = stop at end                                |
| `max_output` | double | `12`    | Max voltage                                         |
| `overturn`   | bool   | `false` | Allow sacrificing speed for sharper turns           |

```cpp
moveToPoint(0, 24, 1, 3000);           // Drive forward to (0, 24)
moveToPoint(-24, 48, -1, 3000);        // Drive backward to (-24, 48)
moveToPoint(10, 10, 1, 3000, true, 8); // Drive at reduced speed
```

---

### `boomerang(x, y, dir, angle, dlead, timeout, exit, max_output, overturn)`

Drive in a smooth curved path to a target pose (position + heading). Like LemLib's `moveToPose`.

| Parameter | Type   | Default | Description                                                               |
| --------- | ------ | ------- | ------------------------------------------------------------------------- |
| `x`, `y`  | double | —       | Target coordinates                                                        |
| `dir`     | int    | —       | `1` = forward, `-1` = backward                                            |
| `angle`   | double | —       | Target **heading** at the end (degrees)                                   |
| `dlead`   | double | —       | Path curvature: `0.3` = gentle, `0.6` = max curve. **Don't go above 0.6** |
| `timeout` | double | —       | Max time in ms                                                            |

```cpp
boomerang(24, 48, 1, 45, 0.4, 4000);   // Curve forward to (24, 48) ending at 45°
boomerang(-12, 36, -1, 90, 0.3, 3000); // Curve backward
```

---

### `curveCircle(end_angle, radius, timeout, exit, max_output)`

Drive in a circular arc.

| Parameter   | Type   | Default | Description                                                             |
| ----------- | ------ | ------- | ----------------------------------------------------------------------- |
| `end_angle` | double | —       | Target heading at end of arc (degrees)                                  |
| `radius`    | double | —       | Arc radius in inches. **Positive = curve right, negative = curve left** |
| `timeout`   | double | —       | Max time in ms                                                          |

```cpp
curveCircle(90, 18, 3000);    // 90° arc curving right, 18" radius
curveCircle(90, -18, 3000);   // 90° arc curving left, 18" radius
```

---

### `swing(angle, drive_dir, timeout, exit, max_output)`

Pivot turn — one side of the drive stays still while the other moves.

| Parameter   | Type   | Default | Description                    |
| ----------- | ------ | ------- | ------------------------------ |
| `angle`     | double | —       | Target heading (degrees)       |
| `drive_dir` | double | —       | `1` = forward, `-1` = backward |
| `timeout`   | double | —       | Max time in ms                 |

```cpp
swing(45, 1, 2000);    // Swing forward to 45°
swing(-45, -1, 2000);  // Swing backward to -45°
```

---

## Subsystem Controls in Autonomous

### Motors

```cpp
intake_motor.spin(fwd, 12, volt);        // Full speed forward
intake_motor.spin(reverse, 12, volt);    // Full speed reverse
intake_motor.spin(fwd, 8, volt);         // Reduced speed
intake_motor.stop(hold);                 // Stop and hold position
intake_motor.stop(coast);                // Stop and coast

outtake_motor.spin(fwd, 12, volt);
outtake_motor.stop(hold);
```

### Pneumatics

```cpp
descore_piston.set(true);       // Extend
descore_piston.set(false);      // Retract
unloader_piston.set(true);
midscoring_piston.set(true);
```

### Direct Chassis Control

```cpp
driveChassis(6, 6);             // Drive forward at half power
driveChassis(-6, -6);           // Drive backward
driveChassis(6, -6);            // Spin right
stopChassis(hold);              // Stop and hold
stopChassis(coast);             // Stop and coast
```

### Delays

```cpp
wait(500, msec);                // Wait 500ms
```

### Shake Bot (Unstick Balls)

```cpp
shakeBot(2000);                 // Shake left/right for 2 seconds
```

Rapidly wiggles the robot left and right (~5 degrees each way) for the given duration in milliseconds. Useful for dislodging stuck balls during unloading.

| Parameter    | Type | Description                         |
| ------------ | ---- | ----------------------------------- |
| `durationMs` | int  | How long to shake (in milliseconds) |

**Tuning (constants inside the function):**
| Constant | Default | Description |
| ---------------- | ------- | ------------------------------------------------------------------ |
| `SHAKE_VOLTAGE` | `5.7` | Motor voltage per shake. Increase for more aggressive (max `12.0`) |
| `SHAKE_INTERVAL` | `150` | ms per half-cycle. Decrease = faster wiggle, increase = wider sweep |

**Example — shake while unloading:**

```cpp
unloader_piston.set(true);
shakeBot(2000);         // wiggle for 2 seconds to unstick balls
unloader_piston.set(false);
```

---

## Motion Chaining (exit = false)

When `exit = false`, the robot doesn't stop at the end of a move. This lets you chain motions together smoothly:

```cpp
// Without chaining (robot stops between each move):
driveTo(24, 3000);
driveTo(24, 3000);

// With chaining (smooth, no stop between):
driveTo(24, 3000, false);  // Don't stop — immediately start next move
driveTo(24, 3000);          // This one stops at the end
```

Chaining is faster and smoother. Use it when you don't need precision between moves — like driving to a general area before scoring.

---

## Tips for Writing Good Autonomous Routines

1. **Start simple.** Get 2-3 moves working perfectly before adding more.
2. **Use generous timeouts** at first (3000-5000ms). Tighten them later once moves are consistent.
3. **Test on the actual field.** Coordinates and distances in your head won't match reality.
4. **Use distance sensor resets** mid-auton to fix odometry drift (see `DISTANCE-SENSOR-GUIDE.md`).
5. **Lower `max_output`** for precise, short moves. Full speed (12V) is for long drives.
6. **Add `wait(100, msec)`** between moves if the robot seems to skip actions — it gives the PID time to settle.

---

## Adding a New Autonomous Routine

1. **Write the function** in `custom/src/autonomous.cpp`:

   ```cpp
   void myNewAuton() {
     x_pos = 0; y_pos = 0; correct_angle = 0;
     driveTo(24, 3000);
     turnToAngle(90, 2000);
   }
   ```

2. **Declare it** in `custom/include/autonomous.h`:

   ```cpp
   void myNewAuton();
   ```

3. **Add it to the selector** in `custom/src/user.cpp`:
   ```cpp
   case 7:
     myNewAuton();
     break;
   ```
