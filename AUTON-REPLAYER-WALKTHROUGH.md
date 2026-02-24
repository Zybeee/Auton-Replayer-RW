# Position-Based Auton Recorder Implementation

## Changes Made

- Created `custom/include/position_replay.h` and `custom/src/position_replay.cpp` to handle recording and replaying autonomous routes.
- Modified `custom/src/user.cpp` to include recording controller mappings:
  - `UP` starts recording.
  - `DOWN` stops recording.
  - `LEFT` begins playback via a spawned `vex::thread`.
  - Added an `if (!is_replaying)` block around driver control to prevent motor interference during playback.
- Modified `custom/src/autonomous.cpp` to add a new `replayAuton` routine.
- Modified `custom/include/autonomous.h` to declare `replayAuton()`.
- Added `replayAuton()` to `case 7` of the `runAutonomous` switch statement in `user.cpp`.

## What Was Not Carried Over

The original implementation had a `midscoring_mode` logic built into the recorder itself. However, since the driver code already manages mid-scoring effectively, the recorder maps directly to the `target.buttons` memory footprint. Therefore, during playback, it operates exactly as the user's joystick controls would operate.

## Verification Steps

Please follow the verification plan attached to the implementation plan:

### Phase 1: Odometry Sanity Check (CRITICAL)

1. Drive forward exactly 24 inches along a tape line.
2. Dump `[x_pos, y_pos]` to the Brain Screen using `Brain.Screen.print`.
3. **Expected**: One axis reads ~24, the other reads ~0. If it differs drastically, evaluate the tracking wheel configurations in `robot-config.cpp`.
4. Turn 90° in place. Confirm `getInertialHeading()` reads roughly 90 or -90 to lock down the sign convention.

### Phase 2: Square Test

1. Press `UP` on the controller to record a 3ft × 3ft square with intermittent intake.
2. Press `DOWN` to stop/save recording.
3. Start playback from the same starting position using `LEFT`.
4. The goal is a smooth continuous motion with minimal drift (≤ 3.0–4.0 inches). `UP+DOWN` during playback will execute an abort and restore the joystick control instantly.

All tasks finished successfully. Try building to test!
