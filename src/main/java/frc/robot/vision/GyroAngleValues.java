package frc.robot.vision;

/**
 * a record represents the gyro's angle passed into the limelight's NT tables, to the robot_orientation_set entry. its values found in
 * limelightlib sourcecode.
 */
public record GyroAngleValues(double yaw, double yawRate, double pitch, double pitchRate, double roll, double rollRate) {}
