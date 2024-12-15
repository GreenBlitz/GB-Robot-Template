package frc.robot.vision;

/**
 * A record that represents the gyro's angle which is passed into the limelight's network tables, to the robot_orientation_set entry. Its values
 * were found in limelightlib sourcecode. The limelight is using it to calculate the BotPose2 position.
 */
public record GyroAngleValues(double yaw, double yawRate, double pitch, double pitchRate, double roll, double rollRate) {}
