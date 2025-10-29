package frc.robot.statemachine;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.constants.field.Field;
import frc.robot.Robot;
import frc.robot.poseestimator.IPoseEstimator;
import frc.robot.scoringhelpers.ScoringHelpers;
import frc.robot.subsystems.swerve.Swerve;
import frc.utils.pose.PoseUtil;

public class PositionTargets {

	private final IPoseEstimator poseEstimator;
	private final Swerve swerve;

	public static final Pose2d NET_OPENING_SUPERSTRUCTURE_POSITION_METERS = new Pose2d(0.07, 0.07, Rotation2d.fromDegrees(2));
	public static final Pose2d NET_SCORING_POSITION_METERS = new Pose2d(0.07, 0.07, Rotation2d.fromDegrees(10));

	public PositionTargets(Robot robot) {
		this.poseEstimator = robot.getPoseEstimator();
		this.swerve = robot.getSwerve();
	}

	public double getDistanceToReef() {
		return poseEstimator.getEstimatedPose().getTranslation().getDistance(Field.getCoralPlacement(ScoringHelpers.getTargetBranch(), true));
	}

	private static final Translation2d MIDDLE_OF_NET_SCORING_RANGE = new Translation2d(7.578, 6.03885);
	private static final Translation2d NET_SCORING_RANGE_TOLERANCE = new Translation2d(0.035, 2.01295);


	public boolean isReadyToScoreNet() {
		return PoseUtil.isAtTranslation(
			poseEstimator.getEstimatedPose().getTranslation(),
			Field.getAllianceRelative(MIDDLE_OF_NET_SCORING_RANGE, true, true),
			NET_SCORING_RANGE_TOLERANCE
		);
	}


	private static final Pose2d PROCESSOR_RELATIVE_SCORING_POSITION = new Pose2d(0.05, 0.05, Rotation2d.fromDegrees(6));
	private static final Pose2d PROCESSOR_RELATIVE_SCORING_DEADBANDS = new Pose2d(3.8, 0.5, Rotation2d.fromRadians(2));

	public boolean isReadyToScoreProcessor() {
		return PoseUtil.isAtPoseAngleRelative(
			Field.getProcessor().getRotation(),
			ScoringHelpers.getAllianceRelativeProcessorScoringPose(),
			poseEstimator.getEstimatedPose(),
			swerve.getAllianceRelativeVelocity(),
			PROCESSOR_RELATIVE_SCORING_POSITION,
			PROCESSOR_RELATIVE_SCORING_DEADBANDS,
			"/isAtProcessorScoringPose"
		);
	}


	public static final Translation2d CLOSE_SUPERSTRUCTURE_ZONE_LENGTH_AND_WIDTH = new Translation2d(0.6, 1.03);

	public boolean isReadyToCloseSuperstructure() {
		Rotation2d reefAngle = Field.getReefSideMiddle(ScoringHelpers.getTargetReefSide()).getRotation();

		Translation2d reefRelativeReefSideMiddle = Field.getReefSideMiddle(ScoringHelpers.getTargetReefSide())
			.rotateBy(reefAngle.unaryMinus())
			.getTranslation();
		Translation2d reefRelativeRobotPose = poseEstimator.getEstimatedPose().rotateBy(reefAngle.unaryMinus()).getTranslation();
		return !PoseUtil.isAtTranslation(reefRelativeRobotPose, reefRelativeReefSideMiddle, CLOSE_SUPERSTRUCTURE_ZONE_LENGTH_AND_WIDTH);
	}


	private static final double REEF_OPEN_DISTANCE_METERS = 0.8;
	private static final Pose2d REEF_RELATIVE_OPEN_SUPERSTRUCTURE_POSITION = new Pose2d(
		REEF_OPEN_DISTANCE_METERS,
		1,
		Rotation2d.fromDegrees(15)
	);
	private static final Pose2d REEF_RELATIVE_OPEN_SUPERSTRUCTURE_DEADBANDS = new Pose2d(2, 2, Rotation2d.fromRadians(4));

	public boolean isReadyToOpenSuperstructure() {
		return isNearReef(
			poseEstimator.getEstimatedPose(),
			swerve.getAllianceRelativeVelocity(),
			REEF_OPEN_DISTANCE_METERS,
			REEF_RELATIVE_OPEN_SUPERSTRUCTURE_POSITION,
			REEF_RELATIVE_OPEN_SUPERSTRUCTURE_DEADBANDS
		);
	}

	private static final double ROBOT_SCORING_DISTANCE_FROM_REEF_METERS = 0.59;
	private static final Pose2d REEF_RELATIVE_SCORING_POSITION = new Pose2d(0.15, 0.025, Rotation2d.fromDegrees(3.5));
	private static final Pose2d REEF_RELATIVE_SCORING_DEADBANDS = new Pose2d(0.6, 0.6, Rotation2d.fromRadians(1.5));

	public boolean isReadyToScoreReef() {
		return isNearReef(
			poseEstimator.getEstimatedPose(),
			swerve.getAllianceRelativeVelocity(),
			ROBOT_SCORING_DISTANCE_FROM_REEF_METERS,
			REEF_RELATIVE_SCORING_POSITION,
			REEF_RELATIVE_SCORING_DEADBANDS
		);
	}


	private static boolean isNearReef(
		Pose2d currentPose,
		ChassisSpeeds currentAllianceRelativeSpeeds,
		double distanceFromReefMeters,
		Pose2d tolerance,
		Pose2d deadband
	) {
		return PoseUtil.isAtPoseAngleRelative(
			Field.getReefSideMiddle(ScoringHelpers.getTargetBranch().getReefSide()).getRotation(),
			ScoringHelpers.getRobotBranchScoringPose(ScoringHelpers.getTargetBranch(), distanceFromReefMeters),
			currentPose,
			currentAllianceRelativeSpeeds,
			tolerance,
			deadband,
			"isAtReefScoringPose"
		);
	}

}
