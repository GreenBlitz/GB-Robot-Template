package frc.constants.field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.constants.field.enums.CagePosition;
import frc.constants.field.enums.CoralStationPosition;
import frc.constants.field.enums.ReefBranch;
import frc.constants.field.enums.ReefSide;
import frc.utils.DriverStationUtils;
import frc.utils.math.FieldMath;

public class Field {

	public static final DriverStation.Alliance RELATIVE_FIELD_CONVENTION_ALLIANCE = DriverStation.Alliance.Blue;

	public static final double LENGTH_METERS = 17.548225;
	public static final double WIDTH_METERS = 8.0518;

	public static final double LENGTH_OF_REEF_SIDE_METERS = 0.96;

	private static final Pose2d[] MIDDLE_OF_REEF_SIDES = new Pose2d[] {
		new Pose2d(3.65760, 4.03220, Rotation2d.fromDegrees(0)),
		new Pose2d(4.07349, 3.31191, Rotation2d.fromDegrees(60)),
		new Pose2d(4.90523, 3.31193, Rotation2d.fromDegrees(120)),
		new Pose2d(5.32107, 4.03225, Rotation2d.fromDegrees(180)),
		new Pose2d(4.90519, 4.75254, Rotation2d.fromDegrees(240)),
		new Pose2d(4.07345, 4.75252, Rotation2d.fromDegrees(300))};

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

	public static boolean isFieldConventionAlliance() {
		return DriverStationUtils.getAlliance() == RELATIVE_FIELD_CONVENTION_ALLIANCE;
	}

	private static Pose2d getAllianceRelative(Pose2d pose, boolean mirrorY, boolean mirrorX, boolean invertAngle, boolean mirrorAngle) {
		return isFieldConventionAlliance() ? pose : FieldMath.getMirrored(pose, mirrorY, mirrorX, invertAngle, mirrorAngle);
	}

	private static Translation2d getAllianceRelative(Translation2d translation, boolean mirrorY, boolean mirrorX) {
		return isFieldConventionAlliance() ? translation : FieldMath.getMirrored(translation, mirrorY, mirrorX);
	}

	private static Translation3d getAllianceRelative(Translation3d translation, boolean mirrorY, boolean mirrorX, boolean invertZ) {
		return isFieldConventionAlliance() ? translation : FieldMath.getMirrored(translation, mirrorY, mirrorX, invertZ);
	}

	private static Rotation3d getAllianceRelative(Rotation3d rotation) {
		return isFieldConventionAlliance() ? rotation : FieldMath.getMirroredAngle(rotation);
	}

	private static Pose3d getAllianceRelative(Pose3d pose, boolean mirrorY, boolean mirrorAngle) {
		Translation3d translation3d = getAllianceRelative(pose.getTranslation(), mirrorY, true, false);
		return mirrorAngle ? new Pose3d(translation3d, getAllianceRelative(pose.getRotation())) : new Pose3d(translation3d, pose.getRotation());
	}

	public static Pose2d getMiddleOfReefSide(ReefSide side) {
		return getAllianceRelative(MIDDLE_OF_REEF_SIDES[side.getIndex()], true, true, true, false);
	}

	public static Translation2d getCoralPlacement(ReefBranch branch) {
		return CORAL_BRANCHES[branch.getIndex()];
	}

	public static Translation2d getCage(CagePosition cagePosition) {
		return getAllianceRelative(CAGES[cagePosition.getIndex()], true, false);
	}

	public static Pose2d getProcessor() {
		return getAllianceRelative(PROCESSOR, true, true, true, false);
	}

	public static Pose2d getMiddleOfCoralStation(CoralStationPosition coralStationPosition) {
		return getAllianceRelative(MIDDLE_OF_CORAL_STATIONS[coralStationPosition.getIndex()], false, true, false, true);
	}

}
