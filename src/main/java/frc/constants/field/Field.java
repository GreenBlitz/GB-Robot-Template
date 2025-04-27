package frc.constants.field;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import frc.utils.DriverStationUtil;
import frc.utils.math.AngleTransform;
import frc.utils.math.FieldMath;

public class Field {

	public static final DriverStation.Alliance RELATIVE_FIELD_CONVENTION_ALLIANCE = DriverStation.Alliance.Blue;

	public static boolean isFieldConventionAlliance() {
		return DriverStationUtil.getAlliance() == RELATIVE_FIELD_CONVENTION_ALLIANCE;
	}

	public static final double LENGTH_METERS = 16.54175;
	public static final double WIDTH_METERS = 8.0137;

	public static Pose2d getAllianceRelative(Pose2d pose, boolean mirrorX, boolean mirrorY, AngleTransform angleTransform) {
		return isFieldConventionAlliance() ? pose : FieldMath.mirror(pose, mirrorX, mirrorY, angleTransform);
	}

	public static Translation2d getAllianceRelative(Translation2d translation, boolean mirrorX, boolean mirrorY) {
		return isFieldConventionAlliance() ? translation : FieldMath.mirror(translation, mirrorX, mirrorY);
	}

	public static Translation3d getAllianceRelative(Translation3d translation, boolean mirrorX, boolean mirrorY) {
		return isFieldConventionAlliance() ? translation : FieldMath.mirror(translation, mirrorX, mirrorY);
	}

	public static Rotation3d getAllianceRelative(Rotation3d rotation) {
		return isFieldConventionAlliance() ? rotation : FieldMath.mirrorAngle(rotation);
	}

	public static Rotation2d getAllianceRelative(Rotation2d rotation) {
		return isFieldConventionAlliance() ? rotation : FieldMath.mirrorAngle(rotation, AngleTransform.INVERT);
	}

	public static Pose3d getAllianceRelative(Pose3d pose, boolean mirrorX, boolean mirrorY, boolean mirrorAngle) {
		Translation3d translation3d = getAllianceRelative(pose.getTranslation(), mirrorX, mirrorY);
		return mirrorAngle ? new Pose3d(translation3d, getAllianceRelative(pose.getRotation())) : new Pose3d(translation3d, pose.getRotation());
	}

	public static Pose2d getPointFromCertainDistance(Pose2d point, double distantInMeters) {
		return new Pose2d(
			point.getX() - point.getRotation().getCos() * distantInMeters,
			point.getY() - point.getRotation().getSin() * distantInMeters,
			point.getRotation()
		);
	}

}
