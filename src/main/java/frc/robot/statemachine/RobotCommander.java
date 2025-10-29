package frc.robot.statemachine;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.constants.field.Field;
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
import frc.robot.statemachine.superstructure.Superstructure;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import frc.utils.DriverStationUtil;
import frc.utils.math.AngleTransform;
import frc.utils.math.FieldMath;
import frc.utils.math.ToleranceMath;

import java.util.Set;
import java.util.function.Supplier;

public class RobotCommander extends GBSubsystem {

	private final Robot robot;
	private final Swerve swerve;
	private final Superstructure superstructure;
	private final LEDStateHandler ledStateHandler;
	private final PositionTargets positionTargets;

	private RobotState currentState;
	public boolean keepAlgaeInIntake;

	public boolean netAssist = true;

	public RobotCommander(String logPath, Robot robot) {
		super(logPath);
		this.robot = robot;
		this.swerve = robot.getSwerve();
		this.positionTargets = new PositionTargets(robot);
		this.superstructure = new Superstructure("StateMachine/Superstructure", robot, positionTargets::getDistanceToReef);
		this.currentState = RobotState.STAY_IN_PLACE;

		this.keepAlgaeInIntake = false;
		Trigger resetKeepAlgaeInIntake = new Trigger(
			() -> currentState == RobotState.ALGAE_OUTTAKE_FROM_END_EFFECTOR
				|| currentState == RobotState.ALGAE_OUTTAKE_FROM_INTAKE
				|| currentState == RobotState.NET
				|| currentState == RobotState.PROCESSOR_SCORE
		);
		resetKeepAlgaeInIntake.onTrue(new InstantCommand(() -> keepAlgaeInIntake = false));

		Trigger resetAutoAlgaeRemove = new Trigger(() -> currentState == RobotState.ALGAE_REMOVE);
		resetAutoAlgaeRemove.onTrue(new InstantCommand(() -> ScoringHelpers.isAutoAlgaeRemoveActivated = false));

		Trigger handleBalls = new Trigger(
			() -> superstructure.isAlgaeInAlgaeIntake()
				&& !superstructure.isCoralIn()
				&& (currentState == RobotState.STAY_IN_PLACE || currentState == RobotState.DRIVE)
				&& DriverStationUtil.isTeleop()
				&& !keepAlgaeInIntake
		);
		handleBalls.onTrue(driveWith(RobotState.TRANSFER_ALGAE_TO_END_EFFECTOR));

		CANdleWrapper caNdleWrapper = new CANdleWrapper(IDs.CANdleIDs.CANDLE, LEDConstants.NUMBER_OF_LEDS, "candle");
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

	public boolean isRunningIndependently() {
		return superstructure.isRunningIndependently() || swerve.getCommandsBuilder().isRunningIndependently();
	}

	public void initializeDefaultCommand() {
		setDefaultCommand(
			new DeferredCommand(
				() -> new ConditionalCommand(
					asSubsystemCommand(Commands.none(), "Disabled"),
					endState(currentState),
					this::isRunningIndependently
				),
				Set.of(this)
			)
		);
	}

	public boolean isReadyToScore() {
		return superstructure.getTargetChecks().isReadyToScore() && positionTargets.isReadyToScoreReef();
	}

	public boolean isReadyForNetForAuto() {
		return positionTargets.isReadyToScoreNet();
	}


	public Command driveWith(RobotState state, Command command) {
		Command swerveDriveCommand = swerve.getCommandsBuilder().driveByDriversInputs(state.getSwerveState());
		Command wantedCommand = command.deadlineFor(swerveDriveCommand);
		return asSubsystemCommand(wantedCommand, state);
	}

	public Command driveWith(RobotState state) {
		return driveWith(state, superstructure.setState(state));
	}


	public Command reefAutomation() {
		Supplier<Command> fullySuperstructureScore = () -> new SequentialCommandGroup(
			superstructure.armPreScore()
				.alongWith(ledStateHandler.setState(LEDState.MOVE_TO_POSE))
				.until(positionTargets::isReadyToOpenSuperstructure),
			superstructure.preScore()
				.alongWith(ledStateHandler.setState(LEDState.IN_POSITION_TO_OPEN_ELEVATOR))
				.until(superstructure.getTargetChecks()::isPreScoreReady),
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

	public Command reefAutomationForAutonomous(PathPlannerPath path) {
		Command fullySuperstructureScore = new SequentialCommandGroup(
			superstructure.elevatorOpening(),
			superstructure.armPreScore()
				.alongWith(ledStateHandler.setState(LEDState.MOVE_TO_POSE))
				.until(positionTargets::isReadyToOpenSuperstructure),
			superstructure.preScore()
				.alongWith(ledStateHandler.setState(LEDState.IN_POSITION_TO_OPEN_ELEVATOR))
				.until(superstructure.getTargetChecks()::isPreScoreReady),
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

	public Command reefAutomationThenAlgaeRemove() {
		return new SequentialCommandGroup(
			reefAutomation(),
			driveWith(RobotState.HOLD_ALGAE).until(positionTargets::isReadyToCloseSuperstructure),
			driveWith(RobotState.ALGAE_REMOVE)
		);
	}

	public Command processorAutomation() {
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
				).until(positionTargets::isReadyToScoreProcessor),
				new ParallelCommandGroup(
					superstructure.processorScore(),
					swerve.getCommandsBuilder().drive(() -> StateMachineConstants.SWERVE_POWERS_TO_PROCESSOR)
				).withTimeout(StateMachineConstants.TIME_TO_RELEASE_ALGAE_TO_PROCESSOR)
			),
			RobotState.PROCESSOR_SCORE
		);
	}

	public Command intakeAutomation() {
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
			RobotState.INTAKE
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
							PositionTargets.NET_OPENING_SUPERSTRUCTURE_POSITION_METERS
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
							PositionTargets.NET_OPENING_SUPERSTRUCTURE_POSITION_METERS
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
								PositionTargets.NET_SCORING_POSITION_METERS
							)
						)
						.andThen(swerve.getCommandsBuilder().resetTargetSpeeds()),
					superstructure.preNet().until(superstructure.getTargetChecks()::isPreNetReady)
				)
			),
			RobotState.PRE_NET
		);
	}

	public Command netAutomation() {
		return new SequentialCommandGroup(driveToPreNet(), driveWith(RobotState.NET));
	}


	private Command afterScore() {
		return new DeferredCommand(
			() -> new SequentialCommandGroup(
				driveWith(RobotState.PRE_SCORE, superstructure.preScore()).until(positionTargets::isReadyToCloseSuperstructure),
				driveWith(RobotState.DRIVE)
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


	private Command asSubsystemCommand(Command command, RobotState state) {
		return new ParallelCommandGroup(asSubsystemCommand(command, state.name()), new InstantCommand(() -> currentState = state));
	}

	private Command endState(RobotState state) {
		Command command = switch (state) {
			case STAY_IN_PLACE, CORAL_OUTTAKE -> driveWith(RobotState.STAY_IN_PLACE);
			case
				INTAKE,
				DRIVE,
				ALGAE_OUTTAKE_FROM_END_EFFECTOR,
				PROCESSOR_SCORE,
				ALGAE_OUTTAKE_FROM_INTAKE,
				ELEVATOR_OPENING,
				PROCESSOR_NO_SCORE,
				ALGAE_INTAKE,
				PRE_NET,
				NET ->
				driveWith(RobotState.DRIVE);
			case ALGAE_REMOVE, HOLD_ALGAE, TRANSFER_ALGAE_TO_END_EFFECTOR -> driveWith(RobotState.HOLD_ALGAE);
			case ARM_PRE_SCORE, CLOSE_CLIMB -> driveWith(RobotState.ARM_PRE_SCORE);
			case PRE_SCORE -> driveWith(RobotState.PRE_SCORE);
			case SCORE, SCORE_WITHOUT_RELEASE -> afterScore();
			case PRE_CLIMB -> driveWith(RobotState.PRE_CLIMB);
			case CLIMB_WITHOUT_LIMIT_SWITCH, CLIMB_WITH_LIMIT_SWITCH, MANUAL_CLIMB, EXIT_CLIMB, STOP_CLIMB -> driveWith(RobotState.STOP_CLIMB);
		};

		command.addRequirements(
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

		return command;
	}

}
