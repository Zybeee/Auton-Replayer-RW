#include "vex.h"
#include "utils.h"
#include "pid.h"
#include <ctime>
#include <cmath>
#include <thread>
#include "../include/autonomous.h"
#include "motor-control.h"
#include "../custom/include/position_replay.h"

// ============================================================================
// CONVERSION NOTES (LemLib → RW-Template)
// ============================================================================
// chassis.moveToPoint(x, y, t, {.maxSpeed=S}, false)
//   → moveToPoint(x, y, dir, t, exit, S/127*12, overturn)
//   dir: 1=forward (default), -1=backward (.forwards=false)
//   exit: true=blocking stop (default), false=chain/async (LemLib's last 'false')
//
// chassis.moveToPose(x, y, heading, t, opts)
//   → boomerang(x, y, dir, heading, 0.4, t, exit, max_output)
//   dlead of 0.4 is a reasonable default — tune if paths are too curved/sharp
//
// chassis.turnToHeading(a, t, {}, false)
//   → turnToAngle(a, t, false)  [false = don't stop; chains into next move]
//
// chassis.swingToHeading(a, DriveSide, t)
//   → swing(a, 1, t)  [drive_direction: 1=fwd, -1=rev]
//
// chassis.setPose(x, y, h)
//   → x_pos = x; y_pos = y; correct_angle = h;
//      inertial_sensor.setRotation(h, degrees);
//
// Motor voltage scaling: LemLib 0–127 → RW 0–12v  (multiply by 12.0/127.0)
//   127 → 12.0v   100 → 9.5v   80 → 7.5v   75 → 7.1v
//    70 → 6.6v     60 → 5.7v   50 → 4.7v   40 → 3.8v   30 → 2.8v
//
// .minSpeed has no direct per-call equivalent in RW — use exit=false instead.
// The chaining mechanism keeps the robot moving through the transition.
// ============================================================================

// ----------------------------------------------------------------------------
// HELPER MACRO: reset pose (position + inertial heading)
// ----------------------------------------------------------------------------
#define SET_POSE(px, py, ph) \
    do { x_pos = (px); y_pos = (py); correct_angle = (ph); \
         inertial_sensor.setRotation((ph), degrees); } while(0)

// ============================================================================
// TUNING AUTONOMOUS
// ============================================================================
void tuningAuton() {
    driveTo(24, 3000);
    turnToAngle(90, 2000);
    turnToAngle(0, 2000);
    driveTo(-24, 3000);
}

// ============================================================================
// SKILLS AUTONOMOUS
// ============================================================================
void skills_auton() {

    // --- Segment 1: First unload ---
    SET_POSE(0, 0, 270);

    moveToPoint(-32.5, 5, 1, 1550);
    turnToAngle(180, 800, false);                      // chain into next

    unloader_piston.set(true);
    intake_motor.spin(reverse, 12, volt);
    wait(500, msec);

    moveToPoint(-32.5, -15, 1, 700, true, 9.5);       // maxSpeed 100 → 9.5v
    wait(2100, msec);

    moveToPoint(-32.5, 5, -1, 1550, false, 7.5);      // backwards, chain, maxSpeed 80
    unloader_piston.set(false);
    intake_motor.stop(coast);

    // --- Wall align + position reset ---
    turnToAngle(90, 800);
    moveToPoint(-55, 5, -1, 800, false);               // backwards align; minSpeed→chain
    SET_POSE(0, 0, 90);

    // --- Segment 2: Drive to second unload zone ---
    turnToAngle(0, 800);
    moveToPoint(0, 93, 1, 1800, true, 7.5);            // maxSpeed 80

    turnToAngle(90, 800);
    moveToPoint(-60, 93, -1, 800, false);              // backwards align; chain
    SET_POSE(0, 0, 90);

    // --- Segment 3: Back to goal and score ---
    moveToPoint(17, 0, 1, 900);
    turnToAngle(0, 800);
    moveToPoint(17, -20, -1, 1100, false);             // backwards into goal; chain

    outtake_motor.spin(fwd, 12, volt);
    intake_motor.spin(reverse, 12, volt);
    unloader_piston.set(true);
    wait(2000, msec);

    // Small nudge forward (was left/right_motors.move(40) for ~75ms)
    left_chassis.spin(fwd, 3.8, volt);
    right_chassis.spin(fwd, 3.8, volt);
    wait(75, msec);
    left_chassis.stop(brake);
    right_chassis.stop(brake);

    turnToAngle(0, 500);
    outtake_motor.spin(reverse, 2.8, volt);            // Outtake.move(-30) → 2.8v

    moveToPoint(19, 40, 1, 1100, true, 5.7);           // maxSpeed 60; unload position
    wait(2300, msec);

    // --- Segment 4: Return to goal, second score ---
    boomerang(16.5, -10, -1, 0, 0.4, 1400, false, 12); // moveToPose backwards; chain
    turnToAngle(0, 750);
    moveToPoint(16.5, -60, -1, 900, false, 7.1);       // backwards; maxSpeed 75; chain

    outtake_motor.spin(fwd, 12, volt);
    unloader_piston.set(false);
    wait(2000, msec);

    moveToPoint(16, -3, 1, 1000);
    outtake_motor.stop(coast);
    intake_motor.stop(coast);

    // --- Segment 5: Cross field to other side ---
    turnToAngle(270, 800);
    boomerang(115, -3, -1, 270, 0.4, 2750);
    boomerang(150, -3, -1, 270, 0.4, 1200, false, 4.7); // maxSpeed 50; chain

    // Wall align + position reset (other side)
    SET_POSE(0, 0, 270);

    // --- Segment 6: Third unload (mirrored) ---
    moveToPoint(-19, 0, 1, 1000);
    turnToAngle(0, 800, false);                        // chain; don't stop

    unloader_piston.set(true);
    intake_motor.spin(reverse, 12, volt);
    wait(500, msec);

    moveToPoint(-19, 200, 1, 600, true, 6.6);          // maxSpeed 70; unload
    wait(2400, msec);

    moveToPoint(-19, -5, -1, 1000, false);             // backwards; chain
    unloader_piston.set(false);
    intake_motor.stop(coast);

    // --- Wall align + position reset ---
    turnToAngle(270, 800);
    moveToPoint(100, -5, -1, 1000, false, 5.7);        // backwards align; maxSpeed 60; chain
    SET_POSE(0, 0, 270);

    // --- Segment 7: Swing and go to third goal ---
    // swingToHeading(0, RIGHT, 800, {}, false) → swing to 0, forward, chain
    swing(0, 1, 800, false);

    boomerang(1, -95, -1, 0, 0.4, 2500);               // moveToPose backwards

    turnToAngle(270, 800);
    moveToPoint(1000, -90, -1, 800, false, 5.7);       // backwards align; chain
    SET_POSE(0, 0, 270);

    // --- Segment 8: Score on third goal ---
    moveToPoint(-17, 0, 1, 1000);
    turnToAngle(180, 800);
    moveToPoint(-17, 30, -1, 1250, false, 7.5);        // backwards into goal; chain

    outtake_motor.spin(fwd, 12, volt);
    intake_motor.spin(reverse, 12, volt);
    unloader_piston.set(true);
    wait(2000, msec);
    outtake_motor.spin(reverse, 2.8, volt);

    turnToAngle(180, 500);
    outtake_motor.spin(reverse, 2.8, volt);

    moveToPoint(-17, -40, 1, 1250, true, 5.7);         // maxSpeed 60; unload
    wait(2300, msec);

    moveToPoint(-17, 0, -1, 1400, false);              // backwards; chain
    turnToAngle(180, 750);
    boomerang(-17, 60, -1, 180, 0.4, 1100, false, 6.6); // moveToPose backwards; chain

    outtake_motor.spin(fwd, 12, volt);
    unloader_piston.set(false);
    wait(2300, msec);

    // --- Segment 9: Final repositioning ---
    SET_POSE(0, 0, 0);

    moveToPoint(0, 15, 1, 1000);
    moveToPoint(15, 15, 1, 1000);
    turnToAngle(180, 1000);
    moveToPoint(15, 60, -1, 1500, false, 6.6);         // backwards; chain

    SET_POSE(0, 0, 0);

    turnToAngle(270, 1000, false);                      // chain
    moveToPoint(-25, -5, 1, 1300);
}

// ============================================================================
// LEFT AUTONOMOUS WITH DESCORE
// ============================================================================
void leftAutonDescore() { // 7 ball
    SET_POSE(0, 0, 0);

    descore_piston.set(true);
    intake_motor.spin(reverse, 12, volt);

    moveToPoint(-10, 20, 1, 2500);
    moveToPoint(-30, 0, 1, 2000, false);               // chain

    // turnToPoint equivalent — face (-30, -10); using turnToAngle to approximate
    // Original: chassis.turnToPoint(-30, -10, 800) — faces that absolute coord
    // Since we just drove near (-30,0), face downward (south ~180°)
    turnToAngle(180, 800);

    unloader_piston.set(true);
    wait(300, msec);

    moveToPoint(-30, -30, 1, 800, true, 6.6);          // maxSpeed 70; unload
    wait(400, msec);

    intake_motor.stop(coast);
    moveToPoint(-30, 30, -1, 1500, false, 7.5);        // backwards; maxSpeed 80; chain

    intake_motor.spin(reverse, 12, volt);
    outtake_motor.spin(fwd, 12, volt);
    unloader_piston.set(false);
    descore_piston.set(false);
    wait(2500, msec);

    intake_motor.stop(coast);
    outtake_motor.stop(coast);

    // --- Wall align + position reset ---
    SET_POSE(0, 0, 0);
    moveToPoint(0, 12, 1, 800);
    turnToAngle(90, 800);
    moveToPoint(-7, 12, -1, 800, false);               // backwards align; chain
    turnToAngle(0, 750, false);                        // chain

    SET_POSE(0, 0, 0);
    moveToPoint(0, -30, -1, 9500);                     // backwards; long timeout
}

// ============================================================================
// LEFT AUTONOMOUS (MID SCORING)
// ============================================================================
void leftAuton() { // mid
    SET_POSE(0, 0, 0);

    descore_piston.set(true);
    intake_motor.spin(reverse, 12, volt);

    moveToPoint(-10, 20, 1, 2500);

    // turnToPoint(8, 38, {.forwards=false}) → face away from (8,38), i.e. back toward it
    // Approximate with turnToAngle facing that direction + 180
    boomerang(8, 38, -1, 0, 0.4, 2000, false, 7.5);   // moveToPose backwards; chain

    intake_motor.spin(fwd, 12, volt);
    wait(200, msec);
    intake_motor.spin(reverse, 12, volt);
    outtake_motor.spin(reverse, 12, volt);

    midscoring_piston.set(true);                       // score mid
    wait(2000, msec);

    outtake_motor.stop(coast);
    midscoring_piston.set(false);

    moveToPoint(-35, 10, 1, 2000, false);              // chain
    turnToAngle(183, 1000, false);                     // chain

    unloader_piston.set(true);
    wait(500, msec);

    moveToPoint(-32, -30, 1, 800, true, 6.6);          // maxSpeed 70; unload
    wait(500, msec);

    intake_motor.stop(coast);
    moveToPoint(-32, 30, -1, 2000, false, 7.5);        // backwards; maxSpeed 80; chain

    intake_motor.spin(reverse, 12, volt);
    outtake_motor.spin(fwd, 12, volt);
    unloader_piston.set(false);
    descore_piston.set(false);
    wait(2500, msec);

    intake_motor.stop(coast);
    outtake_motor.stop(coast);

    // Small nudge
    left_chassis.spin(fwd, 3.8, volt);
    right_chassis.spin(fwd, 3.8, volt);
    wait(200, msec);
    left_chassis.stop(brake);
    right_chassis.stop(brake);
    wait(300, msec);

    // swingToHeading(330, RIGHT, 2000) then swingToHeading(15, RIGHT, 1000)
    swing(330, 1, 2000, true);
    swing(15, 1, 1000, false);                         // chain
    wait(300, msec);

    SET_POSE(0, 0, 0);
    moveToPoint(0, 16.7, 1, 1000);
}

// ============================================================================
// RIGHT AUTONOMOUS
// ============================================================================
void rightAuton() {
    SET_POSE(0, 0, 0);

    descore_piston.set(true);
    intake_motor.spin(reverse, 12, volt);

    moveToPoint(10, 20, 1, 2500);
    moveToPoint(30, 0, 1, 2000, false);                // chain

    // turnToPoint(30, -10, 800) → face roughly south (180°)
    turnToAngle(180, 800);

    unloader_piston.set(true);
    wait(300, msec);

    moveToPoint(30, -30, 1, 800, true, 6.6);           // maxSpeed 70; unload
    wait(400, msec);

    intake_motor.stop(coast);
    moveToPoint(30, 30, -1, 1500, false, 7.5);         // backwards; maxSpeed 80; chain

    intake_motor.spin(reverse, 12, volt);
    outtake_motor.spin(fwd, 12, volt);
    unloader_piston.set(false);
    descore_piston.set(false);
    wait(2500, msec);

    intake_motor.stop(coast);
    outtake_motor.stop(coast);

    // --- Wall align + position reset ---
    SET_POSE(0, 0, 0);
    moveToPoint(0, 12, 1, 800);
    turnToAngle(90, 800);
    moveToPoint(-7, 12, -1, 800, false);               // backwards align; chain
    turnToAngle(0, 750, false);                        // chain

    SET_POSE(0, 0, 0);
    moveToPoint(0, -30, -1, 9500);                     // backwards; long timeout
}

// ============================================================================
// SHAKE BOT — unstick balls during unloading
// ============================================================================
/**
 * Shakes the bot left and right (~5 degrees each direction) for the given
 * duration in milliseconds. Useful for unsticking balls during unloading.
 *
 * Usage: shakeBot(2000);  // shake for 2 seconds while unloading
 *
 * Tuning:
 *   SHAKE_VOLTAGE  (5.7v / ~60 out of 127) — increase for more aggressive
 *                   shaking, decrease for gentler. Max is 12.0v.
 *   SHAKE_INTERVAL (150ms) — decrease for faster/tighter wiggling,
 *                   increase for slower/wider sweeps.
 */
void shakeBot(int durationMs) {
    const double SHAKE_VOLTAGE  = 5.7;   // ~60/127 converted to volts
    const int    SHAKE_INTERVAL = 150;   // ms per half-cycle (left or right)

    vex::timer t;
    t.reset();

    while (t.time(msec) < durationMs) {
        // Turn left: left motors backward, right motors forward
        left_chassis.spin(reverse, SHAKE_VOLTAGE, volt);
        right_chassis.spin(fwd, SHAKE_VOLTAGE, volt);
        wait(SHAKE_INTERVAL, msec);

        // Turn right: left motors forward, right motors backward
        left_chassis.spin(fwd, SHAKE_VOLTAGE, volt);
        right_chassis.spin(reverse, SHAKE_VOLTAGE, volt);
        wait(SHAKE_INTERVAL, msec);
    }

    // Stop and settle
    left_chassis.stop(brake);
    right_chassis.stop(brake);
}

// ============================================================================
// REPLAY AUTON
// ============================================================================
void replayAuton() {
    SET_POSE(0, 0, 0); // resets odometry and inertial
    wait(100, msec);   // IMU settle delay
    positionReplay.playback();
}
