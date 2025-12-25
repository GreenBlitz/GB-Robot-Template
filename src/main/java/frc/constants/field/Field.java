package frc.constants.field;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.utils.DriverStationUtil;
import frc.utils.math.AngleTransform;
import frc.utils.math.FieldMath;

public class Field {

	public static final Translation2d Tower = new Translation2d(6.5, 4);

	public static final double TOWER_SIDES_DISTANCE_FROM_MIDDLE_METERS = 0.27;

	public static final DriverStation.Alliance RELATIVE_FIELD_CONVENTION_ALLIANCE = DriverStation.Alliance.Blue;

	public static boolean isFieldConventionAlliance() {
		return DriverStationUtil.getAlliance() == RELATIVE_FIELD_CONVENTION_ALLIANCE;
	}

	public static final double LENGTH_METERS = 17.548225;
	public static final double WIDTH_METERS = 8.0518;

	public static Pose2d getAllianceRelative(Pose2d pose2d) {
		return new Pose2d(getAllianceRelative(pose2d.getTranslation()), getAllianceRelative(pose2d.getRotation()));
	}

	public static Translation2d getAllianceRelative(Translation2d translation) {
		return isFieldConventionAlliance() ? translation : FieldMath.mirror(translation, true, true);
	}

	public static Rotation2d getAllianceRelative(Rotation2d rotation) {
		return isFieldConventionAlliance() ? rotation : FieldMath.transformAngle(rotation, AngleTransform.INVERT);
	}

	public static Pose3d getAllianceRelative(Pose3d pose) {
		return new Pose3d(getAllianceRelative(pose.getTranslation()), getAllianceRelative(pose.getRotation()));
	}

	public static Translation3d getAllianceRelative(Translation3d translation) {
		return isFieldConventionAlliance() ? translation : FieldMath.mirror(translation, true, true);
	}

	public static Rotation3d getAllianceRelative(Rotation3d rotation) {
		return isFieldConventionAlliance()
			? rotation
			: FieldMath.transformAngle(rotation, AngleTransform.KEEP, AngleTransform.KEEP, AngleTransform.INVERT);
	}

}
