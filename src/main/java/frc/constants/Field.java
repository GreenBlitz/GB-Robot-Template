package frc.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.utils.DriverStationUtils;
import frc.utils.mirror.MirrorMath;

public class Field {

	public static final DriverStation.Alliance RELATIVE_FIELD_CONVENTION_ALLIANCE = DriverStation.Alliance.Blue;

	public static boolean isFieldConventionAlliance() {
		return DriverStationUtils.getAlliance() == RELATIVE_FIELD_CONVENTION_ALLIANCE;
	}

	public static final double LENGTH_METERS = 16.54175;
	public static final double WIDTH_METERS = 8.21055;

	public static final Pose2d ALLIANCE_RELATIVE_STAGE = isFieldConventionAlliance() ? new Pose2d(3.0734, WIDTH_METERS/2, new Rotation2d()) : new Pose2d(MirrorMath.getMirroredX(3.0734), MirrorMath.getMirroredY(3.0), new Rotation2d());
}
