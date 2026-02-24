#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller controller_1 = controller(primary);

// ============================================================================
// DRIVE MOTORS
// ============================================================================
// Format: motor(port, gearSetting, reversed)
// gearSetting: ratio36_1(red), ratio18_1(green), ratio6_1(blue)
// All chassis motors should spin forward when given positive voltage (e.g. driveChassis(12, 12))
motor left_chassis1 = motor(PORT20, ratio6_1, true);
motor left_chassis2 = motor(PORT17, ratio6_1, true);
motor left_chassis3 = motor(PORT18, ratio6_1, false);
motor_group left_chassis = motor_group(left_chassis1, left_chassis2, left_chassis3);
motor right_chassis1 = motor(PORT19, ratio6_1, false);
motor right_chassis2 = motor(PORT16, ratio6_1, false);
motor right_chassis3 = motor(PORT15, ratio6_1, true);
motor_group right_chassis = motor_group(right_chassis1, right_chassis2, right_chassis3);

// ============================================================================
// SENSORS
// ============================================================================
inertial inertial_sensor = inertial(PORT13);

// Tracking wheels (rotation sensor)
// Format: rotation(port, reversed)
// Set to random ports if you don't use tracking wheels
rotation vertical_tracker = rotation(PORT11, true);
rotation horizontal_tracker = rotation(PORT10, true); // Not used, placeholder port

// ============================================================================
// DISTANCE SENSORS — CHANGE PORTS to match your wiring
// ============================================================================
distance back_sensor_left = distance(PORT1);   // CHANGE: left-side back sensor port
distance back_sensor_right = distance(PORT2);  // CHANGE: right-side back sensor port
distance left_sensor = distance(PORT3);        // CHANGE: left side sensor port
distance right_sensor = distance(PORT4);       // CHANGE: right side sensor port

// ============================================================================
// GAME-SPECIFIC DEVICES
// ============================================================================
motor intake_motor = motor(PORT21, ratio6_1, true);   // Blue cartridge, reversed
motor outtake_motor = motor(PORT12, ratio6_1, false);  // Blue cartridge, not reversed

digital_out descore_piston = digital_out(Brain.ThreeWirePort.A);
digital_out midscoring_piston = digital_out(Brain.ThreeWirePort.B);
digital_out unloader_piston = digital_out(Brain.ThreeWirePort.C);

// ============================================================================
// USER-CONFIGURABLE PARAMETERS (CHANGE BEFORE USING THIS TEMPLATE)
// ============================================================================

// Distance between the middles of the left and right wheels of the drive (in inches)
double distance_between_wheels = 11.5;

// motor to wheel gear ratio * wheel diameter (in inches) * pi
// Blue 600rpm motors with 36:48 external gearing, 3.25" new omni wheels
double wheel_distance_in = (36.0 / 48.0) * 3.25 * M_PI;

// ============================================================================
// PID Constants — CHANGE THESE! These are starting estimates, you MUST tune on your robot.
// LemLib values were: lateral kP=4.25 kD=1 | angular kP=0.863 kD=0.235
// RW-Template PID works differently, so these won't match. Tune from scratch.
// ============================================================================
// distance_* : Linear PID for straight driving
// turn_*     : PID for turning in place
// heading_correction_* : PID for heading correction during linear movement
double distance_kp = 1.1, distance_ki = 0.1, distance_kd = 7;       // CHANGE: Tune these!
double turn_kp = 0.3, turn_ki = 0, turn_kd = 2.5;                   // CHANGE: Tune these!
double heading_correction_kp = 0.6, heading_correction_ki = 0, heading_correction_kd = 4; // CHANGE: Tune these!

// ============================================================================
// TRACKING WHEEL CONFIGURATION
// ============================================================================
bool using_horizontal_tracker = false;  // No horizontal tracking wheel
bool using_vertical_tracker = true;     // Vertical tracking wheel is installed

// Vertical distance from center of bot to horizontal tracking wheel (in inches)
// (positive = wheel is behind center, top-down view facing up)
double horizontal_tracker_dist_from_center = 0; // Not used
// Horizontal distance from center of bot to vertical tracking wheel (in inches)
// (positive = wheel is to the right of center, top-down view facing up)
double vertical_tracker_dist_from_center = 0.375;
double horizontal_tracker_diameter = 2.75; // Not used
double vertical_tracker_diameter = 2.75;   // 2.75" omni tracking wheel

// ============================================================================
// DISTANCE SENSOR CONFIGURATION — CHANGE these measurements for your robot
// ============================================================================
// All offsets = distance from sensor to robot center (inches), must be positive
double back_sensor_left_offset = 7.0;    // CHANGE: left back sensor to robot center
double back_sensor_right_offset = 7.0;   // CHANGE: right back sensor to robot center
double back_sensor_spacing = 10.0;       // CHANGE: center-to-center between the two back sensors (inches)
double left_sensor_offset = 6.0;         // CHANGE: left sensor to robot center
double right_sensor_offset = 6.0;        // CHANGE: right sensor to robot center

// ============================================================================
// ADVANCED TUNING (OPTIONAL)
// ============================================================================

bool heading_correction = true; // Use heading correction when the bot is stationary

// Set to true for more accuracy and smoothness, false for more speed
bool dir_change_start = true;   // Less accel/decel due to expecting direction change at start
bool dir_change_end = true;     // Less accel/decel due to expecting direction change at end

double min_output = 10; // Minimum output voltage to motors while chaining movements

// Maximum allowed change in voltage output per 10 msec during movement
double max_slew_accel_fwd = 24;
double max_slew_decel_fwd = 24;
double max_slew_accel_rev = 24;
double max_slew_decel_rev = 24;

// Prevents too much slipping during boomerang movements
// Decrease if there is too much drifting and inconsistency during boomerang
// Increase for more speed during boomerang
double chase_power = 2;

// ============================================================================
// DO NOT CHANGE ANYTHING BELOW
// ============================================================================

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // nothing to initialize
}