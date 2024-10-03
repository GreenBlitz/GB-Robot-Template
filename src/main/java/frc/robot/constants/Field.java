package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.utils.DriverStationUtils;
import frc.utils.mirror.MirrorMath;

public class Field {

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



	public static Pose2d mirrorPositionToOtherSide(Pose2d pose) {
		return new Pose2d(
				LENGTH_METERS - pose.getX(),
				pose.getY(),
				Rotation2d.fromRadians(Math.PI).minus(pose.getRotation())
		);
	}


	public static Rotation2d getMidClimbAngle() {
		if (DriverStationUtils.getAlliance() == RELATIVE_FIELD_CONVENTION_ALLIANCE) {
			return Rotation2d.fromDegrees(180);
		}
		return Rotation2d.fromDegrees(0);
	}

	public static Rotation2d getAMPClimbAngle() {
		if (DriverStationUtils.getAlliance() == RELATIVE_FIELD_CONVENTION_ALLIANCE) {
			return Rotation2d.fromDegrees(-60);
		}
		return Rotation2d.fromDegrees(-120);
	}

	public static Rotation2d getSourceClimbAngle() {
		if (DriverStationUtils.getAlliance() == RELATIVE_FIELD_CONVENTION_ALLIANCE) {
			return Rotation2d.fromDegrees(60);
		}
		return Rotation2d.fromDegrees(120);
	}

	private final static Translation2d BLUE_MID_CLIMB_TRANSLATION = new Translation2d(5.94175, 4);
	public static Translation2d getMidClimbTranslation() {
		if (DriverStationUtils.getAlliance() == RELATIVE_FIELD_CONVENTION_ALLIANCE) {
			return BLUE_MID_CLIMB_TRANSLATION;
		}
		return new Translation2d(MirrorMath.getMirroredX(BLUE_MID_CLIMB_TRANSLATION.getX()), BLUE_MID_CLIMB_TRANSLATION.getY());
	}

	private final static Translation2d BLUE_AMP_CLIMB_TRANSLATION = new Translation2d(4.34175, 5);
	public static Translation2d getAMPClimbTranslation() {
		if (DriverStationUtils.getAlliance() == RELATIVE_FIELD_CONVENTION_ALLIANCE) {
			return BLUE_MID_CLIMB_TRANSLATION;
		}
		return new Translation2d(MirrorMath.getMirroredX(BLUE_MID_CLIMB_TRANSLATION.getX()), BLUE_MID_CLIMB_TRANSLATION.getY());
	}

	private final static Translation2d BLUE_SOURCE_CLIMB_TRANSLATION = new Translation2d(4.34175, 3.2);
	public static Translation2d getSourceClimbTranslation() {
		if (DriverStationUtils.getAlliance() == RELATIVE_FIELD_CONVENTION_ALLIANCE) {
			return BLUE_MID_CLIMB_TRANSLATION;
		}
		return new Translation2d(MirrorMath.getMirroredX(BLUE_MID_CLIMB_TRANSLATION.getX()), BLUE_MID_CLIMB_TRANSLATION.getY());
	}

}
