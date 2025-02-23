package frc.robot.statemachine;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.*;
import frc.constants.field.Field;
import frc.robot.Robot;
import frc.robot.scoringhelpers.ScoringHelpers;
import frc.robot.scoringhelpers.ScoringPathsHelper;
import frc.robot.statemachine.superstructure.Superstructure;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveMath;
import frc.robot.subsystems.swerve.states.SwerveState;
import frc.robot.subsystems.swerve.states.aimassist.AimAssist;
import frc.utils.pose.PoseUtil;

import java.util.Set;
import java.util.function.Supplier;

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
	}

	public Superstructure getSuperstructure() {
		return superstructure;
	}

	public void initializeDefaultCommand() {
		setDefaultCommand(
			new DeferredCommand(
				() -> endState(currentState),
				Set.of(this, superstructure, swerve, robot.getElevator(), robot.getArm(), robot.getEndEffector())
			)
		);
	}

	/**
	 * Check if robot at pose but relative to target branch. Y-axis is vertical to the branch. X-axis is horizontal to the branch So when you
	 * check if robot in place in y-axis its in parallel to the reef side.
	 */
	private boolean isAtReefScoringPose(
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

	private boolean isAtProcessorScoringPose() {
		Rotation2d processorAngle = Field.getProcessor().getRotation();

		Pose2d processorRelativeTargetPose = ScoringHelpers.getAllianceRelativeProcessorScoringPose().rotateBy(processorAngle.unaryMinus());
		Pose2d processorRelativeRobotPose = robot.getPoseEstimator().getEstimatedPose().rotateBy(processorAngle.unaryMinus());

		ChassisSpeeds allianceRelativeSpeeds = swerve.getAllianceRelativeVelocity();
		ChassisSpeeds processorRelativeSpeeds = SwerveMath
			.robotToAllianceRelativeSpeeds(allianceRelativeSpeeds, Field.getAllianceRelative(processorAngle.unaryMinus()));

		return PoseUtil.isAtPose(
			processorRelativeRobotPose,
			processorRelativeTargetPose,
			processorRelativeSpeeds,
			Tolerances.PROCESSOR_RELATIVE_SCORING_POSITION,
			Tolerances.PROCESSOR_RELATIVE_SCORING_DEADBANDS
		);
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

	/**
	 * Checks if the robot is out of the safe zone to close the superstructure
	 */
	public boolean isReadyToCloseSuperstructure() {
		Rotation2d reefAngle = Field.getReefSideMiddle(ScoringHelpers.getTargetReefSide()).getRotation();

		Translation2d reefRelativeReefSideMiddle = Field.getReefSideMiddle(ScoringHelpers.getTargetReefSide())
			.rotateBy(reefAngle.unaryMinus())
			.getTranslation();
		Translation2d reefRelativeRobotPose = robot.getPoseEstimator().getEstimatedPose().rotateBy(reefAngle.unaryMinus()).getTranslation();

		return !PoseUtil
			.isAtTranslation(reefRelativeRobotPose, reefRelativeReefSideMiddle, StateMachineConstants.CLOSE_SUPERSTRUCTURE_LENGTH_AND_WIDTH);
	}

	public boolean isReadyToScore() {
		return superstructure.isReadyToScore()
			&& isAtReefScoringPose(
				StateMachineConstants.ROBOT_SCORING_DISTANCE_FROM_REEF_METERS,
				Tolerances.REEF_RELATIVE_L1_SCORING_POSITION,
				Tolerances.REEF_RELATIVE_L1_SCORING_DEADBANDS,
				Tolerances.REEF_RELATIVE_SCORING_POSITION,
				Tolerances.REEF_RELATIVE_SCORING_DEADBANDS
			);
	}

	public boolean isCloseToNet(double distance) {
		double distanceFromMidXAxis = Math.abs(robot.getPoseEstimator().getEstimatedPose().getTranslation().getX() - Field.LENGTH_METERS / 2);
		return distanceFromMidXAxis < distance;
	}

	public boolean isAtOpenSuperstructureDistanceFromNet() {
		return isCloseToNet(StateMachineConstants.OPEN_SUPERSTRUCTURE_DISTANCE_FROM_NET_METERS);
	}

	public boolean isReadyForNet() {
		return isCloseToNet(StateMachineConstants.SCORE_DISTANCE_FROM_NET_METERS) && superstructure.isReadyForNet();
	}

	public Command setState(RobotState state) {
		return switch (state) {
			case DRIVE -> drive();
			case STAY_IN_PLACE -> stayInPlace();
			case INTAKE -> intake();
			case CORAL_OUTTAKE -> coralOuttake();
			case ALIGN_REEF -> alignReef();
			case ARM_PRE_SCORE -> armPreScore();
			case PRE_SCORE -> preScore();
			case SCORE_WITHOUT_RELEASE -> scoreWithoutRelease();
			case SCORE -> score();
			case ALGAE_REMOVE -> algaeRemove();
			case ALGAE_OUTTAKE -> algaeOuttake();
			case PRE_NET -> preNet();
			case NET_WITHOUT_RELEASE -> netWithoutRelease();
			case NET_WITH_RELEASE -> netWithRelease();
			case PROCESSOR_SCORE -> fullyProcessorScore();
		};
	}

	public Command autoScore() {
		Supplier<Command> fullySuperstructureScore = () -> new SequentialCommandGroup(
			superstructure.armPreScore().until(this::isReadyToOpenSuperstructure),
			superstructure.preScore().until(superstructure::isPreScoreReady),
			superstructure.scoreWithoutRelease().until(this::isReadyToScore),
			superstructure.scoreWithRelease()
		);

		Supplier<Command> driveToPath = () -> swerve.getCommandsBuilder()
			.driveToPath(
				() -> robot.getPoseEstimator().getEstimatedPose(),
				ScoringPathsHelper.getPathByBranch(ScoringHelpers.getTargetBranch()),
				ScoringHelpers
					.getRobotBranchScoringPose(ScoringHelpers.getTargetBranch(), StateMachineConstants.ROBOT_SCORING_DISTANCE_FROM_REEF_METERS)
			);

		return asSubsystemCommand(
			new DeferredCommand(
				() -> new ParallelDeadlineGroup(fullySuperstructureScore.get(), driveToPath.get()),
				Set.of(superstructure, this, swerve, robot.getElevator(), robot.getArm(), robot.getEndEffector())
			),
			RobotState.SCORE
		);
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
		return new SequentialCommandGroup(scoreWithoutRelease().until(this::isReadyToScore), score());
	}

	public Command fullyPreScore() {
		return new SequentialCommandGroup(
			armPreScore().until(this::isReadyToOpenSuperstructure),
			preScore().until(this::isPreScoreReady),
			scoreWithoutRelease()
		);
	}


	public Command fullyProcessorScore() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				swerve.getCommandsBuilder()
					.driveToPose(robot.getPoseEstimator()::getEstimatedPose, ScoringHelpers::getAllianceRelativeProcessorScoringPose),
				new SequentialCommandGroup(superstructure.idle().until(this::isAtProcessorScoringPose), superstructure.algaeOuttake())
			).until(() -> !superstructure.isAlgaeIn()),
			RobotState.PROCESSOR_SCORE
		);
	}

	public Command fullyPreNet() {
		return new SequentialCommandGroup(preNet().until(this::isAtOpenSuperstructureDistanceFromNet), netWithoutRelease());
	}

	public Command fullyNet() {
		return new SequentialCommandGroup(fullyPreNet().until(this::isReadyForNet), netWithRelease());
	}

	private Command drive() {
		return asSubsystemCommand(
			new ParallelCommandGroup(superstructure.idle(), swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)),
			RobotState.DRIVE
		);
	}

	private Command stayInPlace() {
		return asSubsystemCommand(
			new ParallelCommandGroup(superstructure.stayInPlace(), swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)),
			RobotState.STAY_IN_PLACE
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

	private Command coralOuttake() {
		return asSubsystemCommand(
			new ParallelDeadlineGroup(superstructure.outtake(), swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)),
			RobotState.CORAL_OUTTAKE
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

	private Command closeAfterScore() {
		return new DeferredCommand(
			() -> new SequentialCommandGroup(
				new ParallelCommandGroup(
					superstructure.afterScore(),
					swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)
				).until(this::isReadyToCloseSuperstructure),
				drive()
			),
			Set.of(this, superstructure, swerve, robot.getElevator(), robot.getArm(), robot.getEndEffector())
		);
	}

	private Command algaeRemove() {
		return asSubsystemCommand(
			new ParallelDeadlineGroup(
				superstructure.algaeRemove(),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.ALGAE_REMOVE))
			),
			RobotState.ALGAE_REMOVE
		);
	}

	private Command closeAfterAlgaeRemove() {
		return new DeferredCommand(
			() -> new SequentialCommandGroup(
				new ParallelCommandGroup(
					superstructure.postAlgaeRemove(),
					swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)
				).until(this::isReadyToCloseSuperstructure),
				drive()
			),
			Set.of(this, superstructure, swerve, robot.getElevator(), robot.getArm(), robot.getEndEffector())
		);
	}

	private Command algaeOuttake() {
		return asSubsystemCommand(
			new ParallelDeadlineGroup(
				superstructure.algaeOuttake(),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)
			),
			RobotState.ALGAE_OUTTAKE
		);
	}

	private Command preNet() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				superstructure.preNet(),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.NET))
			),
			RobotState.PRE_NET
		);
	}

	private Command netWithoutRelease() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				superstructure.netWithoutRelease(),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.NET))
			),
			RobotState.NET_WITHOUT_RELEASE
		);
	}

	private Command netWithRelease() {
		return asSubsystemCommand(
			new ParallelDeadlineGroup(
				superstructure.netWithRelease(),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.NET))
			),
			RobotState.NET_WITH_RELEASE
		);
	}

	private Command asSubsystemCommand(Command command, RobotState state) {
		return new ParallelCommandGroup(asSubsystemCommand(command, state.name()), new InstantCommand(() -> currentState = state));
	}

	private Command endState(RobotState state) {
		return switch (state) {
			case STAY_IN_PLACE, CORAL_OUTTAKE -> stayInPlace();
			case INTAKE, DRIVE, ALIGN_REEF, ALGAE_OUTTAKE, PROCESSOR_SCORE -> drive();
			case ARM_PRE_SCORE -> armPreScore();
			case PRE_SCORE -> preScore();
			case SCORE, SCORE_WITHOUT_RELEASE -> closeAfterScore();
			case ALGAE_REMOVE -> closeAfterAlgaeRemove();
			case PRE_NET, NET_WITHOUT_RELEASE, NET_WITH_RELEASE -> preNet();
		};
	}

}
