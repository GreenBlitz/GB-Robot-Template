package frc.robot.statemachine;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.*;
import frc.constants.field.Field;
import frc.constants.field.enums.Branch;
import frc.robot.Robot;
import frc.robot.scoringhelpers.ScoringHelpers;
import frc.robot.statemachine.superstructure.ScoreLevel;
import frc.robot.statemachine.superstructure.Superstructure;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveMath;
import frc.robot.subsystems.swerve.states.SwerveState;
import frc.robot.subsystems.swerve.states.aimassist.AimAssist;
import frc.utils.pose.PoseUtil;

import java.util.Set;

public class RobotCommander extends GBSubsystem {

	private final Robot robot;
	private final Swerve swerve;
	private final Superstructure superstructure;

	private RobotState currentState;

	public RobotCommander(String logPath, Robot robot) {
		super(logPath);
		this.robot = robot;
		this.swerve = robot.getSwerve();
		this.superstructure = new Superstructure("StateMachine/Superstructure", robot);
		this.currentState = RobotState.DRIVE;

		setDefaultCommand(
			new DeferredCommand(
				() -> endState(currentState),
				Set.of(this, superstructure, swerve, robot.getElevator(), robot.getArm(), robot.getEndEffector())
			)
		);
	}

	public Superstructure getSuperstructure() {
		return superstructure;
	}


	/**
	 * Check if robot at pose but relative to target branch. Y-axis is vertical to the branch. X-axis is horizontal to the branch So when you
	 * check if robot in place in y-axis its in parallel to the reef side.
	 */
	public boolean isAtReefScoringPose(
		double scoringPoseDistanceFromReefMeters,
		Pose2d l1Tolerances,
		Pose2d l1Deadbands,
		Pose2d tolerances,
		Pose2d deadbands
	) {
		Rotation2d reefAngle = Field.getReefSideMiddle(ScoringHelpers.getTargetBranch().getReefSide()).getRotation();

		Pose2d reefRelativeTargetPose = ScoringHelpers
			.getRobotBranchScoringPose(ScoringHelpers.getTargetBranch(), scoringPoseDistanceFromReefMeters)
			.rotateBy(reefAngle.unaryMinus());
		Pose2d reefRelativeRobotPose = robot.getPoseEstimator().getEstimatedPose().rotateBy(reefAngle.unaryMinus());

		ChassisSpeeds allianceRelativeSpeeds = swerve.getAllianceRelativeVelocity();
		ChassisSpeeds reefRelativeSpeeds = SwerveMath
			.robotToAllianceRelativeSpeeds(allianceRelativeSpeeds, Field.getAllianceRelative(reefAngle.unaryMinus()));

		return switch (ScoringHelpers.targetScoreLevel) {
			case L1 -> PoseUtil.isAtPose(reefRelativeRobotPose, reefRelativeTargetPose, reefRelativeSpeeds, l1Tolerances, l1Deadbands);
			case L2, L3, L4 -> PoseUtil.isAtPose(reefRelativeRobotPose, reefRelativeTargetPose, reefRelativeSpeeds, tolerances, deadbands);
		};
	}

	private boolean isReadyToOpenSuperstructure() {
		return isAtReefScoringPose(
			StateMachineConstants.OPEN_SUPERSTRUCTURE_DISTANCE_FROM_REEF_METERS,
			Tolerances.REEF_RELATIVE_L1_OPEN_SUPERSTRUCTURE_POSITION,
			Tolerances.REEF_RELATIVE_L1_OPEN_SUPERSTRUCTURE_DEADBANDS,
			Tolerances.REEF_RELATIVE_OPEN_SUPERSTRUCTURE_POSITION,
			Tolerances.REEF_RELATIVE_OPEN_SUPERSTRUCTURE_DEADBANDS
		);
	}

	private boolean isPreScoreReady() {
		return superstructure.isPreScoreReady()
			&& isAtReefScoringPose(
				StateMachineConstants.ROBOT_SCORING_DISTANCE_FROM_REEF_METERS,
				Tolerances.REEF_RELATIVE_L1_SCORING_POSITION,
				Tolerances.REEF_RELATIVE_L1_SCORING_DEADBANDS,
				Tolerances.REEF_RELATIVE_SCORING_POSITION,
				Tolerances.REEF_RELATIVE_SCORING_DEADBANDS
			);
	}

	public boolean isReadyToScore() {
		return superstructure.isReadyToScore() && isAtReefScoringPose(
				StateMachineConstants.ROBOT_SCORING_DISTANCE_FROM_REEF_METERS,
				Tolerances.REEF_RELATIVE_L1_SCORING_POSITION,
				Tolerances.REEF_RELATIVE_L1_SCORING_DEADBANDS,
				Tolerances.REEF_RELATIVE_SCORING_POSITION,
				Tolerances.REEF_RELATIVE_SCORING_DEADBANDS
		);
	}

	public Command setState(RobotState state) {
		return switch (state) {
			case DRIVE -> drive();
			case INTAKE -> intake();
			case OUTTAKE -> outtake();
			case ALIGN_REEF -> alignReef();
			case ARM_PRE_SCORE -> armPreScore();
			case PRE_SCORE -> preScore();
			case SCORE_WITHOUT_RELEASE -> scoreWithoutRelease();
			case SCORE -> score();
		};
	}

	public Command fullyScore() {
			return new SequentialCommandGroup(
				armPreScore().until(this::isReadyToOpenSuperstructure),
				preScore().until(this::isPreScoreReady),
				scoreWithoutRelease().until(this::isReadyToScore),
				score()
			);
	}

	public Command scoreForButton() {
		return new SequentialCommandGroup(
			scoreWithoutRelease().until(this::isReadyToScore),
			score()
		);
	}

	public Command fullyPreScore() {
		return new SequentialCommandGroup(
			armPreScore().until(this::isReadyToOpenSuperstructure),
			preScore().until(this::isPreScoreReady),
			scoreWithoutRelease()
		);
	}

	private Command drive() {
		return asSubsystemCommand(
			new ParallelCommandGroup(superstructure.idle(), swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)),
			RobotState.DRIVE
		);
	}

	private Command intake() {
		return asSubsystemCommand(
			new ParallelDeadlineGroup(
				superstructure.intake(),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.CORAL_STATION))
			),
			RobotState.INTAKE
		);
	}

	private Command outtake() {
		return asSubsystemCommand(
			new ParallelCommandGroup(superstructure.outtake(), swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)),
			RobotState.OUTTAKE
		);
	}

	private Command alignReef() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				superstructure.idle(),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.REEF))
			),
			RobotState.ALIGN_REEF
		);
	}

	private Command armPreScore() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				superstructure.armPreScore(),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.BRANCH))
			),
			RobotState.ARM_PRE_SCORE
		);
	}

	private Command preScore() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				superstructure.preScore(),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.BRANCH))
			),
			RobotState.PRE_SCORE
		);
	}

	private Command scoreWithoutRelease() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				superstructure.scoreWithoutRelease(),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.BRANCH))
			),
			RobotState.SCORE_WITHOUT_RELEASE
		);
	}

	private Command score() {
		return asSubsystemCommand(
			new ParallelDeadlineGroup(
				superstructure.scoreWithRelease(),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.BRANCH))
			),
			RobotState.SCORE
		);
	}

	private Command asSubsystemCommand(Command command, RobotState state) {
		return new ParallelCommandGroup(asSubsystemCommand(command, state.name()), new InstantCommand(() -> currentState = state));
	}

	private Command endState(RobotState state) {
		return switch (state) {
			case INTAKE, OUTTAKE, DRIVE, ALIGN_REEF -> drive();
			case ARM_PRE_SCORE -> armPreScore();
			case PRE_SCORE, SCORE, SCORE_WITHOUT_RELEASE -> preScore();
		};
	}

}
