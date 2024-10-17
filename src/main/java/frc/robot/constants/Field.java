package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.utils.DriverStationUtils;
import frc.utils.mirror.MirrorMath;

public class Field {

	public static final double APRIL_TAGS_AVERAGE_HEIGHT_METERS = 1.24;

	public static final DriverStation.Alliance RELATIVE_FIELD_CONVENTION_ALLIANCE = DriverStation.Alliance.Blue;

	public static boolean isFieldConventionAlliance() {
		return DriverStationUtils.getAlliance() == RELATIVE_FIELD_CONVENTION_ALLIANCE;
	}

	public static final double LENGTH_METERS = 16.54175;
	public static final double WIDTH_METERS = 8.0137;


	private static final Translation3d SPEAKER = new Translation3d(0.23, WIDTH_METERS - 2.55, 2.045);

	public static Translation3d getSpeaker() {
		if (DriverStationUtils.getAlliance() == RELATIVE_FIELD_CONVENTION_ALLIANCE) {
			return SPEAKER;
		}
		return new Translation3d(MirrorMath.getMirroredX(SPEAKER.getX()), SPEAKER.getY(), SPEAKER.getZ());
	}

	private static final Rotation2d ANGLE_TO_AMP = Rotation2d.fromDegrees(90);

	public static Rotation2d getAngleToAmp() {
		if (DriverStationUtils.getAlliance() == RELATIVE_FIELD_CONVENTION_ALLIANCE) {
			return ANGLE_TO_AMP;
		}
		return MirrorMath.getMirroredAngle(ANGLE_TO_AMP);
	}

}
