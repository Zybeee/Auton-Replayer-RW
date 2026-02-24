# Distance Sensor Guide — RW-Template

This guide covers how to set up and use the distance sensor position + heading reset system.

---

## Your Sensor Layout

```
              Front (no sensor)
                    ↑
                    |
Left (1 sensor) ← 🤖 → Right (1 sensor)
                    |
              ↓  ↓  ↓
        Back Left  Back Right  (2 sensors)
```

- **Back**: 2 sensors → resets **position AND heading**
- **Left / Right**: 1 sensor each → resets **position only** (with trig correction)

**Total: 4 distance sensors, 4 smart ports used**

---

## Step 1: Configure Ports

In `custom/src/robot-config.cpp`, set the correct ports:

```cpp
distance back_sensor_left = distance(PORT1);   // ← Change to your actual port
distance back_sensor_right = distance(PORT2);  // ← Change to your actual port
distance left_sensor = distance(PORT3);        // ← Change to your actual port
distance right_sensor = distance(PORT4);       // ← Change to your actual port
```

---

## Step 2: Measure and Set Offsets

You need to measure 5 values on your robot. **All measurements are in inches.**

### Back Sensor Offsets

Measure the distance from each back sensor to the **center of the robot** (where the tracking point is). To be specific, it is the exact middle point between your left and right wheels (or tracking wheels), and the middle point between your front and back wheels.

```
           ← spacing →
    [sensorL]       [sensorR]
         ↕ offset       ↕ offset
         ● (robot center)
```

```cpp
double back_sensor_left_offset = 7.0;    // ← Measure this
double back_sensor_right_offset = 7.0;   // ← Measure this
double back_sensor_spacing = 10.0;       // ← Center-to-center between the two back sensors
```

> **Tip:** The offset is the distance **along the back axis** from the sensor face to the robot's tracking center. If both sensors are at the same depth, both offsets will be the same.

> **Tip:** The spacing is measured **horizontally** — the left-to-right distance between the two sensor faces, center-to-center.

### Side Sensor Offsets

```cpp
double left_sensor_offset = 6.0;         // ← Distance from left sensor to robot center
double right_sensor_offset = 6.0;        // ← Distance from right sensor to robot center
```

---

## Step 3: Use in Autonomous

### Back Reset (Dual Sensor — Position + Heading)

Call this when the **back of the robot is near/against a wall**:

```cpp
void leftAuton() {
  x_pos = -48; y_pos = -60; correct_angle = 0;

  // Drive to scoring position
  driveTo(24, 3000);
  intake_motor.spin(fwd, 12, volt);
  wait(1000, msec);

  // Back up to the wall and reset odometry
  driveUntilDistance(back_sensor_left, 3.0, 5.0, false, 3000);  // Back up until 3" from wall
  resetPositionAndHeadingBack();  // Fixes position AND heading

  // Now odometry is accurate again — continue routine
  driveTo(36, 4000);
}
```

**What `resetPositionAndHeadingBack()` does:**

1. Reads both back sensors
2. Calculates exact angle to wall using `atan2(d_right - d_left, spacing)`
3. Corrects the distance using `cos(angle)` for the true perpendicular distance
4. Resets the appropriate axis (X or Y) based on which wall the back faces
5. Resets `correct_angle` (heading) based on the calculated wall angle

### Side Resets (Single Sensor — Position Only)

Call these when the **left or right side** of the robot is near a wall:

```cpp
// Left side near a wall
resetPositionLeft();

// Right side near a wall
resetPositionRight();
```

These use the IMU heading with trig correction (`cos(angle_off_perpendicular)`) for accuracy even when not perfectly perpendicular.

---

## `driveUntilDistance()` — Lining Up to a Wall

Before calling a reset, you want the robot close to the wall. `driveUntilDistance()` drives until a sensor reads below a threshold:

```cpp
driveUntilDistance(sensor, threshold_in, speed, forwards, timeout_ms);
```

| Parameter      | Default | Description                                      |
| -------------- | ------- | ------------------------------------------------ |
| `sensor`       | —       | Which distance sensor to watch                   |
| `threshold_in` | —       | Stop when reading ≤ this (inches)                |
| `speed`        | `6.0`   | Motor voltage (0-12). Lower = less overshoot     |
| `forwards`     | `true`  | `true` = drive forward, `false` = drive backward |
| `timeout_ms`   | `3000`  | Emergency stop time (ms)                         |

```cpp
// Back up slowly until 3 inches from the wall behind you
driveUntilDistance(back_sensor_left, 3.0, 5.0, false, 3000);

// Drive forward until 4 inches from the wall in front
driveUntilDistance(left_sensor, 4.0, 5.0, true, 2000);
```

---

## When to Use Resets

| Situation                         | What to Call                                                |
| --------------------------------- | ----------------------------------------------------------- |
| Backing into a wall (most common) | `driveUntilDistance(...)` → `resetPositionAndHeadingBack()` |
| Side is along a wall              | `resetPositionLeft()` or `resetPositionRight()`             |
| After a long sequence of moves    | Back into nearest wall → dual reset                         |
| At the start of skills autonomous | Back into starting wall → dual reset                        |

### Best Practices

1. **Reset early and often during skills** — odometry drifts more over 60 seconds
2. **Always reset after long sequences** — after 4-5 moves, drift accumulates
3. **Use `driveUntilDistance` before resetting** — gets you close to the wall at a controlled speed
4. **When combining back and side resets, ORDER MATTERS**: ALWAYS do `driveUntilDistance`, THEN `resetPositionAndHeadingBack()`, THEN your side reset (`resetPositionLeft()` or `Right()`). The side reset relies on the heading being perfect to calculate its own trig correction. If you do the side reset first, its calculation will be entirely wrong!
5. **Lower speed for `driveUntilDistance`** — 4-6V is ideal. Full speed = overshoot
6. **Robot should be roughly perpendicular** to the wall (within ~20°). The dual sensor setup corrects for small angle errors, but large angles will be inaccurate

---

## Troubleshooting

| Issue                                                    | Fix                                                                                                |
| -------------------------------------------------------- | -------------------------------------------------------------------------------------------------- |
| Reset puts robot at wrong position                       | Check sensor offsets — they might be measured wrong                                                |
| Reset heading is way off                                 | Check `back_sensor_spacing` — measure center-to-center again                                       |
| "Invalid sensor reading" on brain screen                 | Sensor might not be detecting a wall — make sure it's pointing at a flat surface                   |
| Position resets X when it should reset Y (or vice versa) | Check that the robot is actually facing the wall you think it is — the code uses heading quadrants |
| Side resets are slightly off                             | This is normal for single-sensor resets — they depend on IMU accuracy                              |

---

## How the Math Works (Optional)

### Dual Back Sensor Heading Calculation

```
     ├──── spacing ────┤
     [sensorL]    [sensorR]
       d_left ↕      ↕ d_right
Wall ─────────────────────────
```

```
angle_to_wall = atan2(d_right - d_left, spacing)
```

If both sensors read the same distance, the angle is 0° (perfectly perpendicular). If the right sensor reads farther, the robot is rotated counter-clockwise.

### Trig Correction for Distance

```
true_perpendicular_distance = raw_reading × cos(angle_off_perpendicular)
```

When you're at 5° off perpendicular, `cos(5°) = 0.996` — so the raw reading is only 0.4% too long. At 10°, it's 1.5% too long. The correction handles this automatically.
