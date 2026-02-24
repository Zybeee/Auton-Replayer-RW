using namespace vex;

// Format: extern device deviceName;

extern brain Brain;

// VEXcode devices
extern controller controller_1;
extern motor left_chassis1;
extern motor left_chassis2;
extern motor left_chassis3;
extern motor_group left_chassis;
extern motor right_chassis1;
extern motor right_chassis2;
extern motor right_chassis3;
extern motor_group right_chassis;
extern inertial inertial_sensor;
extern rotation horizontal_tracker;
extern rotation vertical_tracker;

// Distance sensors (set correct ports in robot-config.cpp when ready)
extern distance back_sensor_left;   // Left-side back sensor (for dual heading + position reset)
extern distance back_sensor_right;  // Right-side back sensor (for dual heading + position reset)
extern distance left_sensor;        // Single left sensor (position reset with trig correction)
extern distance right_sensor;       // Single right sensor (position reset with trig correction)

// Game-specific devices
extern motor intake_motor;
extern motor outtake_motor;
extern digital_out descore_piston;
extern digital_out midscoring_piston;
extern digital_out unloader_piston;

// USER-CONFIGURABLE PARAMETERS (CHANGE BEFORE USING THIS TEMPLATE)
extern double distance_between_wheels;
extern double wheel_distance_in;
extern double distance_kp, distance_ki, distance_kd;
extern double turn_kp, turn_ki, turn_kd;
extern double heading_correction_kp, heading_correction_ki, heading_correction_kd;

extern bool using_horizontal_tracker;
extern bool using_vertical_tracker;
extern double horizontal_tracker_dist_from_center;
extern double vertical_tracker_dist_from_center;
extern double horizontal_tracker_diameter;
extern double vertical_tracker_diameter;

// Distance sensor offsets (distance from sensor to robot center, in inches)
extern double back_sensor_left_offset;   // CHANGE: measure from left back sensor to robot center
extern double back_sensor_right_offset;  // CHANGE: measure from right back sensor to robot center
extern double back_sensor_spacing;       // CHANGE: center-to-center distance between the two back sensors (inches)
extern double left_sensor_offset;        // CHANGE: measure from left sensor to robot center
extern double right_sensor_offset;       // CHANGE: measure from right sensor to robot center

// Advanced Tuning Parameters
extern bool heading_correction;
extern bool dir_change_start;
extern bool dir_change_end;
extern double min_output;
extern double max_slew_accel_fwd;
extern double max_slew_decel_fwd;
extern double max_slew_accel_rev;
extern double max_slew_decel_rev;
extern double chase_power;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);