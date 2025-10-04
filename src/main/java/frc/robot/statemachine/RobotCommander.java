package frc.robot.statemachine;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.constants.field.Field;
import frc.constants.field.enums.Branch;
import frc.robot.IDs;
import frc.robot.Robot;
import frc.robot.autonomous.AutonomousConstants;
import frc.robot.autonomous.PathFollowingCommandsBuilder;
import frc.robot.hardware.phoenix6.leds.CANdleWrapper;
import frc.robot.led.LEDConstants;
import frc.robot.led.LEDState;
import frc.robot.led.LEDStateHandler;
import frc.robot.scoringhelpers.ScoringHelpers;
import frc.robot.scoringhelpers.ScoringPathsHelper;
import frc.robot.statemachine.superstructure.ScoreLevel;
import frc.robot.statemachine.superstructure.Superstructure;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveMath;
import frc.robot.subsystems.swerve.states.SwerveState;
import frc.robot.subsystems.swerve.states.aimassist.AimAssist;
import frc.utils.DriverStationUtil;
import frc.utils.math.AngleTransform;
import frc.utils.math.FieldMath;
import frc.utils.math.ToleranceMath;
import frc.utils.pose.PoseUtil;

import java.util.Set;
import java.util.function.Supplier;

public class RobotCommander extends GBSubsystem {

	private final Robot robot;
	private final Swerve swerve;
	private final Superstructure superstructure;

	private final Trigger handleBalls;

	private RobotState currentState;
	public boolean keepAlgaeInIntake;

	private CANdleWrapper caNdleWrapper;
	private LEDStateHandler ledStateHandler;

	public RobotCommander(String logPath, Robot robot) {
		super(logPath);
		this.robot = robot;
		this.swerve = robot.getSwerve();
		this.superstructure = new Superstructure("StateMachine/Superstructure", robot);

		this.currentState = RobotState.STAY_IN_PLACE;
		this.keepAlgaeInIntake = false;


		this.handleBalls = new Trigger(
			() -> superstructure.isAlgaeInAlgaeIntake()
				&& !robot.getEndEffector().isCoralIn()
				&& (currentState == RobotState.STAY_IN_PLACE || currentState == RobotState.DRIVE)
				&& DriverStationUtil.isTeleop()
				&& !keepAlgaeInIntake
		);
		handleBalls.onTrue(transferAlgaeFromIntakeToEndEffector());

		Trigger resetKeepAlgaeInIntake = new Trigger(
			() -> currentState == RobotState.ALGAE_OUTTAKE_FROM_END_EFFECTOR
				|| currentState == RobotState.ALGAE_OUTTAKE_FROM_INTAKE
				|| currentState == RobotState.NET
				|| currentState == RobotState.PROCESSOR_SCORE
		);
		resetKeepAlgaeInIntake.onTrue(new InstantCommand(() -> keepAlgaeInIntake = false));

		this.caNdleWrapper = new CANdleWrapper(IDs.CANdleIDs.CANDLE, LEDConstants.NUMBER_OF_LEDS, "candle");
		this.ledStateHandler = new LEDStateHandler("CANdle", caNdleWrapper);

		initializeDefaultCommand();
	}

	public RobotState getCurrentState() {
		return currentState;
	}

	public Superstructure getSuperstructure() {
		return superstructure;
	}

	public LEDStateHandler getLedStateHandler() {
		return ledStateHandler;
	}

	public void initializeDefaultCommand() {
		setDefaultCommand(
			new DeferredCommand(
				() -> endState(currentState),
				Set.of(
					this,
					superstructure,
					swerve,
					robot.getElevator(),
					robot.getArm(),
					robot.getEndEffector(),
					robot.getLifter(),
					robot.getSolenoid(),
					robot.getPivot(),
					robot.getRollers()
				)
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
			case L1 ->
				PoseUtil.isAtPose(
					reefRelativeRobotPose,
					reefRelativeTargetPose,
					reefRelativeSpeeds,
					l1Tolerances,
					l1Deadbands,
					"/isAtL1ScoringPose"
				);
			case L2, L3, L4 ->
				PoseUtil
					.isAtPose(reefRelativeRobotPose, reefRelativeTargetPose, reefRelativeSpeeds, tolerances, deadbands, "/isAtReefScoringPose");
		};
	}

	private boolean isAtBranchScoringPose(
		Branch targetBranch,
		double scoringPoseDistanceFromReefMeters,
		Pose2d l1Tolerances,
		Pose2d l1Deadbands,
		Pose2d tolerances,
		Pose2d deadbands
	) {
		Rotation2d reefAngle = Field.getReefSideMiddle(targetBranch.getReefSide()).getRotation();

		Pose2d reefRelativeTargetPose = ScoringHelpers.getRobotBranchScoringPose(targetBranch, scoringPoseDistanceFromReefMeters)
			.rotateBy(reefAngle.unaryMinus());
		Pose2d reefRelativeRobotPose = robot.getPoseEstimator().getEstimatedPose().rotateBy(reefAngle.unaryMinus());

		ChassisSpeeds allianceRelativeSpeeds = swerve.getAllianceRelativeVelocity();
		ChassisSpeeds reefRelativeSpeeds = SwerveMath
			.robotToAllianceRelativeSpeeds(allianceRelativeSpeeds, Field.getAllianceRelative(reefAngle.unaryMinus()));

		return switch (ScoringHelpers.targetScoreLevel) {
			case L1 ->
				PoseUtil
					.isAtPose(reefRelativeRobotPose, reefRelativeTargetPose, reefRelativeSpeeds, l1Tolerances, l1Deadbands, "/isAL1ScoringPose");
			case L2, L3, L4 ->
				PoseUtil.isAtPose(
					reefRelativeRobotPose,
					reefRelativeTargetPose,
					reefRelativeSpeeds,
					tolerances,
					deadbands,
					"/isAtBranchScoringPose"
				);
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
			Tolerances.PROCESSOR_RELATIVE_SCORING_DEADBANDS,
			"/isAtProcessorScoringPose"
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

	public boolean isAtBranchScoringPose(Branch branch) {
		return isAtBranchScoringPose(
			branch,
			StateMachineConstants.ROBOT_SCORING_DISTANCE_FROM_REEF_METERS,
			Tolerances.REEF_RELATIVE_L1_SCORING_POSITION,
			Tolerances.REEF_RELATIVE_L1_SCORING_DEADBANDS,
			Tolerances.REEF_RELATIVE_SCORING_POSITION,
			Tolerances.REEF_RELATIVE_SCORING_DEADBANDS
		);
	}

	public boolean isReadyToActivateCoralStationAimAssist() {
		Translation2d robotTranslation = robot.getPoseEstimator().getEstimatedPose().getTranslation();
		Translation2d coralStationSlotTranslation = Field.getCoralStationSlot(ScoringHelpers.getTargetCoralStationSlot(robot)).getTranslation();
		return robotTranslation.getDistance(coralStationSlotTranslation)
			<= StateMachineConstants.DISTANCE_FROM_CORAL_STATION_SLOT_TO_START_AIM_ASSIST_METERS;
	}

	private boolean isCloseToNet() {
		Translation2d middleOfNetScoringRange = new Translation2d(7.578, 6.03885);
		Translation2d netScoringRangeDistancesFromMiddle = new Translation2d(0.035, 2.01295);
		return PoseUtil.isAtTranslation(
			robot.getPoseEstimator().getEstimatedPose().getTranslation(),
			Field.getAllianceRelative(middleOfNetScoringRange, true, true),
			netScoringRangeDistancesFromMiddle
		);
	}

	public boolean isReadyForNetForAuto() {
		return isCloseToNet();
	}

	public Command driveWith(String name, Command command, boolean asDeadline) {
		Command swerveDriveCommand = swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE);
		Command wantedCommand = asDeadline ? command.deadlineFor(swerveDriveCommand) : command.alongWith(swerveDriveCommand);
		return asSubsystemCommand(wantedCommand, name);
	}

	public Command setState(RobotState state) {
		return switch (state) {
			case DRIVE -> drive();
			case STAY_IN_PLACE -> stayInPlace();
			case INTAKE_WITH_AIM_ASSIST -> intakeWithAimAssist();
			case INTAKE_WITHOUT_AIM_ASSIST -> intakeWithoutAimAssist();
			case CORAL_OUTTAKE -> coralOuttake();
			case ALIGN_REEF -> alignReef();
			case ARM_PRE_SCORE -> armPreScore();
			case PRE_SCORE -> preScore();
			case SCORE_WITHOUT_RELEASE -> scoreWithoutRelease();
			case SCORE -> score();
			case ALGAE_REMOVE -> algaeRemove();
			case ALGAE_OUTTAKE_FROM_END_EFFECTOR -> algaeOuttakeFromEndEffector();
			case ALGAE_OUTTAKE_FROM_INTAKE -> algaeOuttakeFromIntake();
			case ALGAE_INTAKE -> algaeIntake();
			case TRANSFER_ALGAE_TO_END_EFFECTOR -> transferAlgaeFromIntakeToEndEffector();
			case AUTO_PRE_NET -> driveToPreNet();
			case PRE_NET -> preNet();
			case NET -> net();
			case PROCESSOR_SCORE -> fullyProcessorScore();
			case PRE_CLIMB_WITH_AIM_ASSIST -> preClimbWithAimAssist();
			case PRE_CLIMB_WITHOUT_AIM_ASSIST -> preClimbWithoutAimAssist();
			case CLIMB_WITHOUT_LIMIT_SWITCH -> climbWithoutLimitSwitch();
			case CLIMB_WITH_LIMIT_SWITCH -> climbWithLimitSwitch();
			case MANUAL_CLIMB -> manualClimb();
			case EXIT_CLIMB -> exitClimb();
			case STOP_CLIMB -> stopClimb();
			case CLOSE_CLIMB -> closeClimb();
			case HOLD_ALGAE -> holdAlgae();
		};
	}

	public Command autoScore() {
		Supplier<Command> fullySuperstructureScore = () -> new SequentialCommandGroup(
			superstructure.armPreScore().alongWith(ledStateHandler.setState(LEDState.MOVE_TO_POSE)).until(this::isReadyToOpenSuperstructure),
			superstructure.preScore()
				.alongWith(ledStateHandler.setState(LEDState.IN_POSITION_TO_OPEN_ELEVATOR))
				.until(superstructure::isPreScoreReady),
			superstructure.scoreWithoutRelease()
				.alongWith(ledStateHandler.setState(LEDState.OPENING_SUPERSTRUCTURE))
				.until(this::isReadyToScore),
			superstructure.scoreWithRelease().deadlineFor(ledStateHandler.setState(LEDState.IN_POSITION_TO_SCORE))
		);

		Supplier<Command> driveToPath = () -> swerve.getCommandsBuilder()
			.driveToPath(
				() -> robot.getPoseEstimator().getEstimatedPose(),
				ScoringPathsHelper.getPathByBranch(ScoringHelpers.getTargetBranch()),
				AutonomousConstants.getRealTimeConstraints(swerve)
			);

		return asSubsystemCommand(
			new DeferredCommand(
				() -> new ParallelDeadlineGroup(fullySuperstructureScore.get(), driveToPath.get()),
				Set.of(
					this,
					superstructure,
					swerve,
					robot.getElevator(),
					robot.getArm(),
					robot.getEndEffector(),
					robot.getLifter(),
					robot.getSolenoid(),
					robot.getPivot(),
					robot.getRollers()
				)
			),
			RobotState.SCORE
		);
	}

	public Command autoScoreForAutonomous(PathPlannerPath path) {
		Command fullySuperstructureScore = new SequentialCommandGroup(
			superstructure.elevatorOpening(),
			superstructure.armPreScore().alongWith(ledStateHandler.setState(LEDState.MOVE_TO_POSE)).until(this::isReadyToOpenSuperstructure),
			superstructure.preScore()
				.alongWith(ledStateHandler.setState(LEDState.IN_POSITION_TO_OPEN_ELEVATOR))
				.until(superstructure::isPreScoreReady),
			superstructure.scoreWithoutRelease()
				.alongWith(ledStateHandler.setState(LEDState.OPENING_SUPERSTRUCTURE))
				.until(this::isReadyToScore),
			superstructure.scoreWithRelease().deadlineFor(ledStateHandler.setState(LEDState.IN_POSITION_TO_SCORE))
		);

		Command driveToPath = swerve.asSubsystemCommand(
			PathFollowingCommandsBuilder.followPath(path)
				.andThen(
					swerve.getCommandsBuilder()
						.moveToPoseByPID(
							() -> robot.getPoseEstimator().getEstimatedPose(),
							ScoringHelpers.getRobotBranchScoringPose(
								ScoringHelpers.getTargetBranch(),
								StateMachineConstants.ROBOT_SCORING_DISTANCE_FROM_REEF_METERS
							)
						)
				),
			"Auto Score Autonomous"
		);

		return new ParallelDeadlineGroup(fullySuperstructureScore, driveToPath);
	}

	public Command autoScoreThenAlgaeRemove() {
		return new DeferredCommand(
			() -> new SequentialCommandGroup(autoScore(), holdAlgae().until(this::isReadyToCloseSuperstructure), algaeRemove()),
			Set.of(
				this,
				superstructure,
				swerve,
				robot.getElevator(),
				robot.getArm(),
				robot.getEndEffector(),
				robot.getLifter(),
				robot.getSolenoid(),
				robot.getPivot(),
				robot.getRollers()
			)
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
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					superstructure.processorWithoutRelease(),
					swerve.getCommandsBuilder()
						.driveToPose(
							robot.getPoseEstimator()::getEstimatedPose,
							ScoringHelpers::getAllianceRelativeProcessorScoringPose,
							AutonomousConstants.getRealTimeConstraints(swerve)
						)
				).until(this::isAtProcessorScoringPose),
				new ParallelCommandGroup(
					superstructure.processorScore(),
					swerve.getCommandsBuilder().drive(() -> StateMachineConstants.SWERVE_POWERS_TO_PROCESSOR)
				).withTimeout(StateMachineConstants.TIME_TO_RELEASE_ALGAE_TO_PROCESSOR)
			),
			RobotState.PROCESSOR_SCORE
		);
	}

	public Command autoNet() {
		return new SequentialCommandGroup(driveToPreNet(), net());
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

	private Command intakeWithAimAssist() {
		return asSubsystemCommand(
			new ParallelDeadlineGroup(
				superstructure.intake(),
				swerve.getCommandsBuilder()
					.driveToPose(
						() -> robot.getPoseEstimator().getEstimatedPose(),
						() -> ScoringHelpers.getIntakePose2d(robot),
						AutonomousConstants.getRealTimeConstraints(swerve)
					)
			),
			RobotState.INTAKE_WITH_AIM_ASSIST
		);
	}

	private Command intakeWithoutAimAssist() {
		return asSubsystemCommand(
			new ParallelDeadlineGroup(superstructure.intake(), swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)),
			RobotState.INTAKE_WITHOUT_AIM_ASSIST
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

	private Command algaeRemove() {
		return asSubsystemCommand(
			new ParallelDeadlineGroup(
				superstructure.algaeRemove(),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.ALGAE_REMOVE)),
				new StartEndCommand(() -> {}, () -> ScoringHelpers.isAutoAlgaeRemoveActivated = false)
			),
			RobotState.ALGAE_REMOVE
		);
	}

	private Command algaeOuttakeFromEndEffector() {
		return asSubsystemCommand(
			new ParallelDeadlineGroup(
				superstructure.algaeOuttakeFromEndEffector(),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)
			),
			RobotState.ALGAE_OUTTAKE_FROM_END_EFFECTOR
		);
	}

	private Command algaeOuttakeFromIntake() {
		return asSubsystemCommand(
			new ParallelDeadlineGroup(
				superstructure.algaeOuttakeFromIntake(),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)
			),
			RobotState.ALGAE_OUTTAKE_FROM_INTAKE
		);
	}

	private Command algaeIntake() {
		return asSubsystemCommand(
			new ParallelDeadlineGroup(
				superstructure.algaeIntake(),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.ALGAE_INTAKE))
			),
			RobotState.ALGAE_INTAKE
		);
	}

	private Command transferAlgaeFromIntakeToEndEffector() {
		return asSubsystemCommand(
			new ParallelDeadlineGroup(
				superstructure.transferAlgaeFromIntakeToEndEffector(),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)
			),
			RobotState.TRANSFER_ALGAE_TO_END_EFFECTOR
		);
	}


	private Command preNet() {
		return asSubsystemCommand(
			new ParallelCommandGroup(superstructure.preNet(), swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)),
			RobotState.PRE_NET
		);
	}

	private Command driveToPreNet() {
		Pose2d netEdgeOpenSuperstructurePosition = FieldMath.mirror(
			new Pose2d(
				StateMachineConstants.NET_SCORING_OPEN_SUPERSTRUCTURE_X_POSITION_METERS,
				StateMachineConstants.MIN_NET_SCORING_Y_POSITION,
				new Rotation2d()
			),
			!Field.isOnBlueSide(robot.getPoseEstimator().getEstimatedPose().getTranslation()),
			!Field.isFieldConventionAlliance(),
			Field.isOnBlueSide(robot.getPoseEstimator().getEstimatedPose().getTranslation()) ? AngleTransform.KEEP : AngleTransform.INVERT
		);
		Supplier<Pose2d> openSuperstructurePosition = () -> FieldMath.mirror(
			new Pose2d(
				StateMachineConstants.NET_SCORING_OPEN_SUPERSTRUCTURE_X_POSITION_METERS,
				robot.getPoseEstimator().getEstimatedPose().getY(),
				new Rotation2d()
			),
			!Field.isOnBlueSide(robot.getPoseEstimator().getEstimatedPose().getTranslation()),
			false,
			Field.isOnBlueSide(robot.getPoseEstimator().getEstimatedPose().getTranslation()) ? AngleTransform.KEEP : AngleTransform.INVERT
		);
		Supplier<Pose2d> scoringPosition = () -> FieldMath.mirror(
			new Pose2d(StateMachineConstants.SCORE_NET_X_POSITION_METERS, robot.getPoseEstimator().getEstimatedPose().getY(), new Rotation2d()),
			!Field.isOnBlueSide(robot.getPoseEstimator().getEstimatedPose().getTranslation()),
			false,
			Field.isOnBlueSide(robot.getPoseEstimator().getEstimatedPose().getTranslation()) ? AngleTransform.KEEP : AngleTransform.INVERT
		);

		return asSubsystemCommand(
			new SequentialCommandGroup(
				swerve.getCommandsBuilder()
					.driveToPose(
						robot.getPoseEstimator()::getEstimatedPose,
						() -> netEdgeOpenSuperstructurePosition,
						AutonomousConstants.getRealTimeConstraints(swerve)
					)
					.until(
						() -> ToleranceMath.isNear(
							netEdgeOpenSuperstructurePosition,
							robot.getPoseEstimator().getEstimatedPose(),
							Tolerances.NET_OPENING_SUPERSTRUCTURE_POSITION_METERS
						)
					)
					.onlyIf(
						() -> Field.isFieldConventionAlliance()
							? robot.getPoseEstimator().getEstimatedPose().getY() < StateMachineConstants.MIN_NET_SCORING_Y_POSITION
							: robot.getPoseEstimator().getEstimatedPose().getY() > StateMachineConstants.MIN_NET_SCORING_Y_POSITION
					),
				PathFollowingCommandsBuilder
					.pathfindToPose(
						openSuperstructurePosition.get(),
						AutonomousConstants.getRealTimeConstraints(swerve),
						StateMachineConstants.VELOCITY_BETWEEN_OPEN_SUPERSTRUCTURE_AND_SCORE_TO_NET_METERS_PER_SECOND
					)
					.until(
						() -> ToleranceMath.isNear(
							openSuperstructurePosition.get(),
							robot.getPoseEstimator().getEstimatedPose(),
							Tolerances.NET_OPENING_SUPERSTRUCTURE_POSITION_METERS
						)
					),
				new ParallelCommandGroup(
					swerve.getCommandsBuilder()
						.driveToPose(
							robot.getPoseEstimator()::getEstimatedPose,
							scoringPosition,
							new PathConstraints(
								StateMachineConstants.MAX_VELOCITY_WHILE_ELEVATOR_L4_METERS_PER_SECOND,
								StateMachineConstants.MAX_ACCELERATION_WHILE_ELEVATOR_L4_METERS_PER_SECOND_SQUARED,
								StateMachineConstants.MAX_VELOCITY_WHILE_ELEVATOR_L4_ROTATION2D_PER_SECOND.getRadians(),
								StateMachineConstants.MAX_ACCELERATION_WHILE_ELEVATOR_L4_ROTATION2D_PER_SECOND_SQUARED.getRadians()
							)
						)
						.until(
							() -> ToleranceMath.isNear(
								scoringPosition.get(),
								robot.getPoseEstimator().getEstimatedPose(),
								Tolerances.NET_SCORING_POSITION_METERS
							)
						)
						.andThen(swerve.getCommandsBuilder().resetTargetSpeeds()),
					superstructure.preNet().until(superstructure::isPreNetReady)
				)
			),
			RobotState.AUTO_PRE_NET
		);
	}

	private Command net() {
		return asSubsystemCommand(
			new ParallelDeadlineGroup(
				superstructure.netWithRelease(),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)
			),
			RobotState.NET
		);
	}

	private Command holdAlgae() {
		return asSubsystemCommand(
			new ParallelCommandGroup(superstructure.holdAlgae(), swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)),
			RobotState.HOLD_ALGAE
		);
	}

	private Command preClimbWithAimAssist() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				superstructure.preClimb(),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.CAGE))
			),
			RobotState.PRE_CLIMB_WITH_AIM_ASSIST
		);
	}

	private Command preClimbWithoutAimAssist() {
		return asSubsystemCommand(
			new ParallelCommandGroup(superstructure.preClimb(), swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)),
			RobotState.PRE_CLIMB_WITHOUT_AIM_ASSIST
		);
	}

	private Command climbWithoutLimitSwitch() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				superstructure.climbWithoutLimitSwitch(),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)
			),
			RobotState.CLIMB_WITHOUT_LIMIT_SWITCH
		);
	}

	private Command climbWithLimitSwitch() {
		return asSubsystemCommand(
			new ParallelCommandGroup(
				superstructure.climbWithLimitSwitch(),
				swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)
			),
			RobotState.CLIMB_WITH_LIMIT_SWITCH
		);
	}

	private Command manualClimb() {
		return asSubsystemCommand(
			new ParallelCommandGroup(superstructure.manualClimb(), swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)),
			RobotState.MANUAL_CLIMB
		);
	}

	private Command exitClimb() {
		return asSubsystemCommand(
			new ParallelCommandGroup(superstructure.exitClimb(), swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)),
			RobotState.EXIT_CLIMB
		);
	}

	public Command stopClimb() {
		return asSubsystemCommand(
			new ParallelCommandGroup(superstructure.stopClimb(), swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)),
			RobotState.STOP_CLIMB
		);
	}

	public Command closeClimb() {
		return asSubsystemCommand(
			new ParallelDeadlineGroup(superstructure.closeClimb(), swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)),
			RobotState.CLOSE_CLIMB
		);
	}

	private Command afterScore() {
		return new DeferredCommand(
			() -> new SequentialCommandGroup(
				(ScoringHelpers.targetScoreLevel == ScoreLevel.L4
					? driveWith("Soft close l4", superstructure.softCloseL4(), true)
					: driveWith("pre score hold l2 l3", superstructure.preScore(), false).until(this::isReadyToCloseSuperstructure)),
				drive()
			),
			Set.of(
				this,
				superstructure,
				swerve,
				robot.getElevator(),
				robot.getArm(),
				robot.getEndEffector(),
				robot.getLifter(),
				robot.getSolenoid(),
				robot.getPivot(),
				robot.getRollers()
			)
		);
	}

	private Command afterNet() {
		return new DeferredCommand(
			() -> new SequentialCommandGroup(driveWith("Soft close net", getSuperstructure().softCloseNet(), true), drive()),
			Set.of(
				this,
				superstructure,
				swerve,
				robot.getElevator(),
				robot.getArm(),
				robot.getEndEffector(),
				robot.getLifter(),
				robot.getSolenoid(),
				robot.getPivot(),
				robot.getRollers()
			)
		);
	}

	private Command asSubsystemCommand(Command command, RobotState state) {
		return new ParallelCommandGroup(asSubsystemCommand(command, state.name()), new InstantCommand(() -> currentState = state));
	}

	private Command endState(RobotState state) {
		return switch (state) {
			case STAY_IN_PLACE, CORAL_OUTTAKE -> stayInPlace();
			case
				INTAKE_WITH_AIM_ASSIST,
				INTAKE_WITHOUT_AIM_ASSIST,
				DRIVE,
				ALIGN_REEF,
				ALGAE_OUTTAKE_FROM_END_EFFECTOR,
				PROCESSOR_SCORE,
				ALGAE_OUTTAKE_FROM_INTAKE,
				ALGAE_INTAKE ->
				drive();
			case AUTO_PRE_NET, PRE_NET, NET -> afterNet();
			case ALGAE_REMOVE, HOLD_ALGAE, TRANSFER_ALGAE_TO_END_EFFECTOR -> holdAlgae();
			case ARM_PRE_SCORE, CLOSE_CLIMB -> armPreScore();
			case PRE_SCORE -> preScore();
			case SCORE, SCORE_WITHOUT_RELEASE -> afterScore();
			case PRE_CLIMB_WITH_AIM_ASSIST -> preClimbWithAimAssist();
			case PRE_CLIMB_WITHOUT_AIM_ASSIST -> preClimbWithoutAimAssist();
			case CLIMB_WITHOUT_LIMIT_SWITCH, CLIMB_WITH_LIMIT_SWITCH, MANUAL_CLIMB, EXIT_CLIMB, STOP_CLIMB -> stopClimb();
		};
	}

}
