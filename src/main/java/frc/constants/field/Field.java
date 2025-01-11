package frc.constants.field;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import frc.utils.DriverStationUtils;
import frc.utils.math.MirrorMath;

public class Field {

	public static final DriverStation.Alliance RELATIVE_FIELD_CONVENTION_ALLIANCE = DriverStation.Alliance.Blue;

	public static boolean isFieldConventionAlliance() {
		return DriverStationUtils.getAlliance() == RELATIVE_FIELD_CONVENTION_ALLIANCE;
	}

	private static Pose2d getAllianceRelativePose2d(Pose2d pose2d, boolean mirrorY, boolean mirrorX, boolean invertAngle) {
		return isFieldConventionAlliance() ? pose2d : MirrorMath.getMirrored(pose2d, mirrorY, mirrorX, invertAngle);
	}

	private static Translation2d getAllianceRelativeTranslation2d(Translation2d translation2d, boolean invertY) {
		return isFieldConventionAlliance() ? translation2d : MirrorMath.getMirrored(translation2d, invertY);
	}

	private static Translation3d getAllianceRelativeTranslation3d(Translation3d translation3d, boolean invertZ) {
		return isFieldConventionAlliance() ? translation3d : MirrorMath.getMirrored(translation3d, invertZ);
	}

	private static Pose3d getAllianceRelativePose3d(Pose3d pose3d, boolean mirrorAngle) {
		Translation3d translation3d = getAllianceRelativeTranslation3d(pose3d.getTranslation(), false);
		return mirrorAngle ? new Pose3d(translation3d, pose3d.getRotation().unaryMinus()) : new Pose3d(translation3d, pose3d.getRotation());
	}

	public static final double LENGTH_METERS = 16.54175;
	public static final double WIDTH_METERS = 8.0137;

}
