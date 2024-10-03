package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.utils.DriverStationUtils;
import frc.utils.mirror.MirrorMath;

import java.util.List;

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


	private final static Pose2d BLUE_MID_CLIMB = new Pose2d(5.94175, 4, Rotation2d.fromDegrees(180));

	public static Pose2d getMidClimb() {
		if (DriverStationUtils.getAlliance() == RELATIVE_FIELD_CONVENTION_ALLIANCE) {
			return BLUE_MID_CLIMB;
		}
		return new Pose2d(MirrorMath.getMirroredX(BLUE_MID_CLIMB.getX()), BLUE_MID_CLIMB.getY(), Rotation2d.fromDegrees(0));
	}

	private final static Pose2d BLUE_AMP_CLIMB = new Pose2d(4.34175, 5, Rotation2d.fromDegrees(-60));

	public static Pose2d getAMPClimb() {
		if (DriverStationUtils.getAlliance() == RELATIVE_FIELD_CONVENTION_ALLIANCE) {
			return BLUE_AMP_CLIMB;
		}
		return new Pose2d(MirrorMath.getMirroredX(BLUE_AMP_CLIMB.getX()), BLUE_AMP_CLIMB.getY(), Rotation2d.fromDegrees(-120));
	}

	private final static Pose2d BLUE_SOURCE_CLIMB = new Pose2d(4.34175, 3.2, Rotation2d.fromDegrees(60));

	public static Pose2d getSourceClimb() {
		if (DriverStationUtils.getAlliance() == RELATIVE_FIELD_CONVENTION_ALLIANCE) {
			return BLUE_SOURCE_CLIMB;
		}
		return new Pose2d(MirrorMath.getMirroredX(BLUE_SOURCE_CLIMB.getX()), BLUE_SOURCE_CLIMB.getY(), Rotation2d.fromDegrees(120));
	}

	public static Pose2d getClosetClimb(Pose2d robot) {
		return robot.nearest(List.of(getMidClimb(), getAMPClimb(), getSourceClimb()));
	}

}
