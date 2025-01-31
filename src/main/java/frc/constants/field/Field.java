package frc.constants.field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.constants.field.enums.*;
import frc.utils.DriverStationUtil;
import frc.utils.math.AngleTransform;
import frc.utils.math.FieldMath;

public class Field {

	public static final DriverStation.Alliance RELATIVE_FIELD_CONVENTION_ALLIANCE = DriverStation.Alliance.Blue;

	public static boolean isFieldConventionAlliance() {
		return DriverStationUtil.getAlliance() == RELATIVE_FIELD_CONVENTION_ALLIANCE;
	}


	public static final double LENGTH_METERS = 17.548225;
	public static final double WIDTH_METERS = 8.0518;

	private static final Translation2d MIDDLE_OF_REEF = new Translation2d(4.48934, 4.03225);

	public static final double LENGTH_OF_REEF_SIDE_METERS = 0.96;

	private static final Pose2d[] MIDDLE_OF_REEF_SIDES = new Pose2d[] {
		new Pose2d(3.65760, 4.03220, Rotation2d.fromDegrees(0)),
		new Pose2d(4.07349, 3.31191, Rotation2d.fromDegrees(60)),
		new Pose2d(4.90523, 3.31193, Rotation2d.fromDegrees(120)),
		new Pose2d(5.32107, 4.03225, Rotation2d.fromDegrees(180)),
		new Pose2d(4.90519, 4.75254, Rotation2d.fromDegrees(-120)),
		new Pose2d(4.07345, 4.75252, Rotation2d.fromDegrees(-60))};

	private static final Translation2d[] CORAL_BRANCHES = new Translation2d[] {
		new Translation2d(3.71123, 4.19654),
		new Translation2d(3.71008, 3.86792),
		new Translation2d(3.95799, 3.44052),
		new Translation2d(4.24201, 3.27522),
		new Translation2d(4.73610, 3.27621),
		new Translation2d(5.02126, 3.43953),
		new Translation2d(5.26745, 3.86792),
		new Translation2d(5.26859, 4.19654),
		new Translation2d(5.02069, 4.62394),
		new Translation2d(4.73667, 4.78924),
		new Translation2d(4.24258, 4.78825),
		new Translation2d(3.93194, 4.62493)};

	private static final Translation2d[] CAGES = new Translation2d[] {
		new Translation2d(8.77412, 7.26599),
		new Translation2d(8.77412, 6.17538),
		new Translation2d(8.77412, 5.08476)};

	public static final Translation2d BARGE_CENTER = new Translation2d(LENGTH_METERS / 2, WIDTH_METERS / 2);

	private static final Pose2d PROCESSOR = new Pose2d(5.98744, 0.00749, Rotation2d.fromDegrees(270));

	private static final Pose2d[] MIDDLE_OF_CORAL_STATIONS = new Pose2d[] {
		new Pose2d(0.84319, 0.65078, Rotation2d.fromDegrees(-126)),
		new Pose2d(0.84319, 7.41395, Rotation2d.fromDegrees(126))};

	public static final double WIDTH_OF_FEEDER_METERS = 1.9304;


	public static Translation2d getMiddleOfReef() {
		return getAllianceRelative(MIDDLE_OF_REEF, true, false);
	}

	public static Pose2d getMiddleOfReefSide(ReefSide side) {
		return getAllianceRelative(MIDDLE_OF_REEF_SIDES[side.getIndex()], true, true, AngleTransform.INVERT);
	}

	public static Translation2d getCoralPlacement(Branch branch) {
		return getAllianceRelative(CORAL_BRANCHES[branch.getIndex()], true, true);
	}

	public static Translation2d getCage(Cage cage) {
		return getAllianceRelative(CAGES[cage.getIndex()], false, true);
	}

	public static Pose2d getProcessor() {
		return getAllianceRelative(PROCESSOR, true, true, AngleTransform.INVERT);
	}

	public static Pose2d getMiddleOfCoralStation(CoralStation coralStation) {
		return getAllianceRelative(MIDDLE_OF_CORAL_STATIONS[coralStation.getIndex()], true, true, AngleTransform.INVERT);
	}


	private static Pose2d getAllianceRelative(Pose2d pose, boolean mirrorX, boolean mirrorY, AngleTransform angleTransform) {
		return isFieldConventionAlliance() ? pose : FieldMath.mirror(pose, mirrorX, mirrorY, angleTransform);
	}

	private static Translation2d getAllianceRelative(Translation2d translation, boolean mirrorX, boolean mirrorY) {
		return isFieldConventionAlliance() ? translation : FieldMath.mirror(translation, mirrorX, mirrorY);
	}

	private static Translation3d getAllianceRelative(Translation3d translation, boolean mirrorX, boolean mirrorY) {
		return isFieldConventionAlliance() ? translation : FieldMath.mirror(translation, mirrorX, mirrorY);
	}

	private static Rotation3d getAllianceRelative(Rotation3d rotation) {
		return isFieldConventionAlliance() ? rotation : FieldMath.mirrorAngle(rotation);
	}

	private static Pose3d getAllianceRelative(Pose3d pose, boolean mirrorX, boolean mirrorY, boolean mirrorAngle) {
		Translation3d translation3d = getAllianceRelative(pose.getTranslation(), mirrorX, mirrorY);
		return mirrorAngle ? new Pose3d(translation3d, getAllianceRelative(pose.getRotation())) : new Pose3d(translation3d, pose.getRotation());
	}

}
