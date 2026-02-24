# PID Tuning Guide — RW-Template

This guide covers how to tune all **3 PID controllers** in RW-Template to get accurate, consistent autonomous movement.

---

## Overview: The 3 PID Controllers

| Controller                 | Variables                                                                 | What It Controls                                              |
| -------------------------- | ------------------------------------------------------------------------- | ------------------------------------------------------------- |
| **Distance PID**           | `distance_kp`, `distance_ki`, `distance_kd`                               | Straight-line driving (how far to go)                         |
| **Turn PID**               | `turn_kp`, `turn_ki`, `turn_kd`                                           | Turning in place (how far to rotate)                          |
| **Heading Correction PID** | `heading_correction_kp`, `heading_correction_ki`, `heading_correction_kd` | Keeping straight while driving / holding heading when stopped |

All values are in `custom/src/robot-config.cpp`. **Tune them in the order listed above** — distance first, turn second, heading correction last.

---

## How PID Works (Quick Refresher)

- **kP (Proportional)** — "How hard to push based on how far away I am." Higher = more aggressive, but too high = overshoot/oscillation.
- **kI (Integral)** — "Accumulate small errors over time and push harder." Fixes steady-state error (robot stops 0.5" short). **Usually keep this low or 0.**
- **kD (Derivative)** — "Slow down when approaching the target." Dampens oscillation. Higher = smoother stops but too high = sluggish response.

---

## Setup

1. Set `auton_selected = 1` in `custom/src/user.cpp` → runs `tuningAuton()`
2. Edit `tuningAuton()` in `custom/src/autonomous.cpp` for each test
3. Run the autonomous, observe, adjust PID values in `robot-config.cpp`, rebuild, repeat

> **Tip:** The Brain screen draws a real-time graph during turns — use it to visualize overshoot and oscillation.

---

## Part 1: Distance PID (`distance_kp`, `distance_ki`, `distance_kd`)

### What These Control

When you call `driveTo(24, 3000)`, this PID calculates how much voltage to send to the motors based on how far the robot has traveled vs. the target distance.

### Starting Values

```cpp
double distance_kp = 1.1, distance_ki = 0.1, distance_kd = 7;
```

### Tuning Process

#### Step 1: Tune kP (set kI and kD to 0 first)

```cpp
double distance_kp = 0.5, distance_ki = 0, distance_kd = 0;
```

Test code:

```cpp
void tuningAuton() {
  driveTo(24, 3000);  // Drive 24 inches forward
}
```

- **Robot doesn't reach target** → increase kP
- **Robot overshoots and comes back** → decrease kP
- **Robot oscillates back and forth** → kP is way too high, cut it in half
- **Goal:** Robot gets close to 24 inches but may overshoot slightly

#### Step 2: Add kD to stop oscillation

```cpp
double distance_kp = 1.0, distance_ki = 0, distance_kd = 3;
```

- If robot still overshoots → increase kD
- If robot is sluggish/takes too long to arrive → decrease kD
- **Goal:** Robot arrives at 24 inches cleanly with no oscillation

#### Step 3: Add kI only if needed

Only add kI if the robot **consistently stops 0.5-1 inch short**. Most teams don't need kI for distance.

```cpp
double distance_kp = 1.0, distance_ki = 0.05, distance_kd = 3;
```

### Validation Tests (ALL must pass)

Don't just test one distance! Run all of these and adjust until they're all accurate:

```cpp
void tuningAuton() {
  // Test 1: Short distance forward
  driveTo(12, 3000);
}
```

```cpp
void tuningAuton() {
  // Test 2: Medium distance forward
  driveTo(24, 3000);
}
```

```cpp
void tuningAuton() {
  // Test 3: Long distance forward
  driveTo(48, 3000);
}
```

```cpp
void tuningAuton() {
  // Test 4: Backward
  driveTo(-24, 3000);
}
```

```cpp
void tuningAuton() {
  // Test 5: Forward then backward (tests stopping accuracy)
  driveTo(24, 3000);
  wait(500, msec);
  driveTo(-24, 3000);
}
```

**Acceptable accuracy: ±1 inch** for all tests. If short distances work but long ones don't (or vice versa), your kP is probably off.

---

## Part 2: Turn PID (`turn_kp`, `turn_ki`, `turn_kd`)

### What These Control

When you call `turnToAngle(90, 2000)`, this PID calculates motor voltage based on the difference between current heading and target heading.

### Starting Values

```cpp
double turn_kp = 0.3, turn_ki = 0, turn_kd = 2.5;
```

### Tuning Process

#### Step 1: Tune kP (set kI and kD to 0)

```cpp
double turn_kp = 0.2, turn_ki = 0, turn_kd = 0;
```

Test code:

```cpp
void tuningAuton() {
  turnToAngle(90, 2000);  // Turn to 90 degrees
}
```

- **Robot doesn't reach 90°** → increase kP
- **Robot overshoots past 90° and comes back** → decrease kP
- **Robot oscillates around 90°** → kP too high
- **Goal:** Robot gets close to 90° but may wobble

#### Step 2: Add kD

```cpp
double turn_kp = 0.3, turn_ki = 0, turn_kd = 1.5;
```

- If robot overshoots → increase kD
- If robot takes forever to settle → decrease kD
- **Goal:** Robot snaps to 90° cleanly

#### Step 3: Add kI only if needed

Only if the robot consistently undershoots by 1-3°.

### Validation Tests (ALL must pass)

This is the critical part — **a PID that only works at 90° is useless.** Test all of these:

```cpp
void tuningAuton() {
  // Test 1: 90° right
  turnToAngle(90, 2000);
}
```

```cpp
void tuningAuton() {
  // Test 2: 90° left
  turnToAngle(-90, 2000);
}
```

```cpp
void tuningAuton() {
  // Test 3: 180° (half turn)
  turnToAngle(180, 2000);
}
```

```cpp
void tuningAuton() {
  // Test 4: 270° (three-quarter turn)
  turnToAngle(270, 3000);
}
```

```cpp
void tuningAuton() {
  // Test 5: 360° (full rotation)
  turnToAngle(360, 3000);
}
```

```cpp
void tuningAuton() {
  // Test 6: Small angle (hardest to tune)
  turnToAngle(30, 2000);
}
```

```cpp
void tuningAuton() {
  // Test 7: Sequential turns (tests heading tracking)
  turnToAngle(90, 2000);
  wait(300, msec);
  turnToAngle(0, 2000);    // Turn back
  wait(300, msec);
  turnToAngle(-90, 2000);  // Turn the other way
  wait(300, msec);
  turnToAngle(0, 2000);    // Turn back to start
}
```

```cpp
void tuningAuton() {
  // Test 8: 45° — common in actual autons
  turnToAngle(45, 2000);
  wait(300, msec);
  turnToAngle(135, 2000);
  wait(300, msec);
  turnToAngle(0, 2000);
}
```

**Acceptable accuracy: ±2°** for all tests. If 90° works but 270° doesn't, your kD may be too low (the robot coasts past on longer turns because it picks up too much speed).

### Common Issues

| Symptom                                | Fix                                                     |
| -------------------------------------- | ------------------------------------------------------- |
| Accurate at 90° but overshoots at 270° | Increase kD — robot needs more braking for longer turns |
| Undershoots small angles (30°, 45°)    | Increase kP slightly — not enough initial push          |
| Oscillates on all angles               | kP too high, decrease by 20%                            |
| Takes forever to settle                | kD too high, decrease by 20%                            |
| Always 2-3° short                      | Add small kI (0.01-0.05)                                |

---

## Part 3: Heading Correction PID (`heading_correction_kp`, `heading_correction_ki`, `heading_correction_kd`)

### What These Control

This PID has two jobs:

1. **During `driveTo()`**: Keeps the robot driving straight (corrects left/right drift)
2. **During `correctHeading()`**: Holds the robot's heading when it's stopped (resists being pushed)

### Starting Values

```cpp
double heading_correction_kp = 0.6, heading_correction_ki = 0, heading_correction_kd = 4;
```

### Why Tune This Last

It depends on distance PID being solid first. If distance PID is wrong, the corrections from heading PID will look off too.

### Tuning Process

**Tune using a long straight drive** — short drives don't show enough drift.

```cpp
void tuningAuton() {
  driveTo(48, 5000);  // Long drive — watch if it curves left or right
}
```

#### If the robot curves/drifts:

- Increase `heading_correction_kp` — more aggressive correction
- If it starts weaving left-right → too aggressive, increase `heading_correction_kd`

#### If the robot drives straight but wobbles:

- `heading_correction_kp` is too high, decrease it

### Validation Tests

```cpp
void tuningAuton() {
  // Test 1: Long drive forward (should be perfectly straight)
  driveTo(48, 5000);
}
```

```cpp
void tuningAuton() {
  // Test 2: Long drive backward
  driveTo(-48, 5000);
}
```

```cpp
void tuningAuton() {
  // Test 3: Drive, turn, drive (heading correction should keep second drive straight)
  driveTo(24, 3000);
  turnToAngle(90, 2000);
  driveTo(24, 3000);
}
```

```cpp
void tuningAuton() {
  // Test 4: Square test (the ultimate test)
  // Robot should end up close to where it started
  driveTo(24, 3000);
  turnToAngle(90, 2000);
  driveTo(24, 3000);
  turnToAngle(180, 2000);
  driveTo(24, 3000);
  turnToAngle(270, 2000);
  driveTo(24, 3000);
  turnToAngle(360, 2000);
}
```

**The square test is the gold standard.** If your robot ends up close to where it started (within ~2 inches), all 3 PIDs are well-tuned.

---

## Final Checklist

- [ ] Distance PID: Robot drives 12", 24", 48" accurately (±1")
- [ ] Distance PID: Robot drives backward accurately
- [ ] Turn PID: Robot turns to 30°, 45°, 90°, 180°, 270°, 360° accurately (±2°)
- [ ] Turn PID: Sequential turns (back and forth) are accurate
- [ ] Heading Correction: Long drives (48") are straight, no curving
- [ ] Heading Correction: Square test — robot returns to start position (±2")

---

## Quick Reference: What Each Value Does

| Parameter | Too Low                          | Too High                  | Sweet Spot                          |
| --------- | -------------------------------- | ------------------------- | ----------------------------------- |
| **kP**    | Robot falls short / sluggish     | Oscillation / overshoot   | Reaches target, slight overshoot OK |
| **kI**    | Steady-state error (stops short) | Windup / oscillation      | Only add if consistently short      |
| **kD**    | Overshoot / bouncing             | Sluggish / slow to arrive | Clean stop, no oscillation          |
