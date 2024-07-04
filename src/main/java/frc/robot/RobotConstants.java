package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class RobotConstants {

    protected static final Pose2d DEFAULT_POSE = new Pose2d(2, 5, new Rotation2d());

    protected static final Rotation2d ROTATION_TOLERANCE = Rotation2d.fromDegrees(1);
    protected static final Rotation2d ROTATION_VELOCITY_TOLERANCE = Rotation2d.fromRadians(0.05);
    protected static final double TRANSLATION_TOLERANCE_METERS = 0.05;
    protected static final double TRANSLATION_VELOCITY_TOLERANCE = 0.05;

}
