package frc.robot.vision.limelights;

/**
 * a record represents the gyros angle passed into the limelight's NT tables, to the robot_orientation_set entry. its values found in
 * limelightlib sourcecode.
 */
public record GyroAngleValues(double yaw, double yawRate, double pitch, double pitchRate, double roll, double rollRate) {}
