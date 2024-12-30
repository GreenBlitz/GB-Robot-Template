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

	public static final Rotation2d ANGLE_TO_AMP = Rotation2d.fromDegrees(90);

	private static final Pose2d PODIUM = new Pose2d(3.0734, WIDTH_METERS/2, new Rotation2d());

	public static Pose2d getPodium(){
		return isFieldConventionAlliance() ? PODIUM : MirrorMath.getMirroredPose(PODIUM);
	}

	private static final Pose2d CENTER_CLIMB = new Pose2d(5.770626, WIDTH_METERS/2, Rotation2d.fromDegrees(180));

	public static Pose2d getCenterClimb(){
		return isFieldConventionAlliance() ? CENTER_CLIMB : MirrorMath.getMirroredPose(CENTER_CLIMB);
	}

	private static final Pose2d AMP_CLIMB = new Pose2d(4.422013, /*7.219823*/4.883912, Rotation2d.fromDegrees(-57));

	public static Pose2d getAmpClimb() {
		return isFieldConventionAlliance() ? AMP_CLIMB : MirrorMath.getMirroredPose(AMP_CLIMB);
	}

	private static final Pose2d SOURCE_CLIMB = new Pose2d(4.422013, 3.326638, Rotation2d.fromDegrees(57));

	public static Pose2d getSourceClimb(){
		return isFieldConventionAlliance() ? SOURCE_CLIMB : MirrorMath.getMirroredPose(SOURCE_CLIMB);
	}
}
