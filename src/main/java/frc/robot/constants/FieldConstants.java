package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

public class FieldConstants {

    public static final DriverStation.Alliance RELATIVE_FIELD_CONVENTION_ALLIANCE = DriverStation.Alliance.Blue;

    public static final double MINIMUM_TIMESTAMP_ALLIANCE_CHECK = 0.5;

    public static final double LENGTH_METERS = 16.54175;

    public static final double WIDTH_METERS = 8.0137;


    public static final Rotation2d FULL_CYCLE = Rotation2d.fromRotations(1);

    public static final Rotation2d HALF_CYCLE = Rotation2d.fromRotations(0.5);

}
