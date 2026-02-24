# Implement Position-Based Auton Recorder in RW-Drive

## Goal Description

Integrate the user-provided LemLib/PROS Position-Based Auton Recorder into the VEXcode V5 `RW-Drive` project. This will involve adapting the `PositionReplay` class to use VEXcode APIs (`vex::timer`, `std::FILE`, `vex::thread`) and `RW-Drive` odometry (`x_pos`, `y_pos`, `getInertialHeading()`).

## Proposed Changes

### 1. `PositionReplay` Class & Threading Model

#### [NEW] `custom/include/position_replay.h`

- `WaypointFrame` struct (packed): x, y, theta, timestamp, mechanism voltages, button bitflags.
- `PositionReplay` class with `vex::thread` for playback.

#### [NEW] `custom/src/position_replay.cpp`

**Recording**: `recordFrame()` appends to `std::vector<WaypointFrame>` in RAM only. No SD writes during active recording. SD write happens exclusively on `stopRecording()`.

- **State Guarding**: `startRecording()` will explicitly check `if (is_replaying)` and immediately return/rumble an error to prevent silently attempting to record while the playback state is active or failed to clean up. Similarly, `startPlaybackThread()` will check `if (_isRecording)`.

**SD Card I/O**: `Brain.SDcard.isInserted()` for presence check. `fopen("/usd/...")` with per-write error checking. Graceful abort on failure.

**Threading Ownership Model (Playback)**:

- `is_replaying` is set to `true` **before** the playback thread is spawned, not inside it. This closes the one-frame race window where `runDriver()` could still output motor commands between the thread spawn and the flag flip.
- `runDriver()` gates all motor writes and subsystem updates behind `if (!is_replaying)`.
- On playback completion or emergency stop (`UP+DOWN`), the thread sets `is_replaying = false`, restoring driver ownership.

### 2. Mathematics

**A) Coordinate Frame & Heading Sign (CRITICAL)**:

- RW-Drive uses compass-style headings: 0° = forward (+Y), CW positive. `getInertialHeading()` returns this directly.
- Standard `atan2(y, x)` returns CCW-positive from +X axis — **incompatible**.
- **Fix**: Use `atan2(dx, dy)` (swapped args) to produce compass-style target headings.
- Normalize both target and current heading to `[-180, 180]` before error calculation:
  ```cpp
  float error = targetHeading - currentHeading;
  while (error > 180) error -= 360;
  while (error < -180) error += 360;
  ```

**B) Core Failure Mode**: Original PD chased the exact elapsed-time frame. Any mechanical lag caused it to oscillate on a point it already passed.

**C) Time-Shifted Lookahead**: Target `elapsed_time + lookahead_offset_ms` (default 150ms, configurable via `setLookaheadOffset()`). Acts as a spring pulling the robot forward on the path.

**D) Derivative Stabilization**: Measure actual `dt` per iteration. Low-pass filter: `smoothed = 0.7*smoothed + 0.3*raw`. Prevents jitter amplification from VEXos scheduling variance.

### 3. Integration

#### [MODIFY] `custom/src/user.cpp`

- **Odometry guarantee**: `main.cpp` calls `pre_auton()` → `runPreAutonomous()` (spawns tracking threads) before `Competition.drivercontrol()` can ever fire.
- **`recordFrame()` injection**: Inside `runDriver()`'s `while(true)` loop, after joystick reads but before `wait(20, msec)`, gated by `if (!is_replaying)`.
- **Controls**: `UP` = start recording, `DOWN` = stop/save, `LEFT` = start playback.

#### [MODIFY] `custom/src/autonomous.cpp`

- **Replay Auton** (new case in `runAutonomous()`):
  - `SET_POSE(0, 0, 0)` — confirmed this macro resets `x_pos`, `y_pos`, `correct_angle`, AND calls `inertial_sensor.setRotation(0, degrees)`. All four values zeroed.
  - `wait(100, msec)` — IMU settle delay.
  - `positionReplay.playback()` — blocking call.

## Verification Plan

### Phase 1: Odometry Sanity Check (CRITICAL — do this first)

1. Drive forward exactly 24 inches along a tape line. Dump `[x_pos, y_pos]` to Brain Screen.
2. **Expected**: One axis reads ~24, the other reads ~0.
3. **If values are off by >10%**: This is a tracking wheel scaling error (`wheel_diameter` or `gear_ratio` in `robot-config.cpp`), not a PD problem. **Fallback**: Measure actual wheel circumference, recalculate `wheel_diameter` constant, re-run this check until within 5%. No point proceeding to Phase 2 until this passes.
4. Turn 90° in place. Confirm `getInertialHeading()` reads ~90 (CW positive) or ~-90 (CCW positive) to lock down sign convention before any PD math runs.

### Phase 2: Square Test

1. Record a 3ft × 3ft square with intermittent intake.
2. Playback from same starting position.
3. **Drift target**: ≤ 3.0–4.0 inches. Smooth continuous motion (no frame-jerk). `UP+DOWN` abort restores joystick within ≤ 50ms.
