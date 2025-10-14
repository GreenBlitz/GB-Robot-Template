package frc.robot.statemachine;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
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
		handleBalls.onTrue(setState(RobotState.TRANSFER_ALGAE_TO_END_EFFECTOR));

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

	public Command driveWith(String name, Command command) {
		return driveWith(name, command, () -> SwerveState.DEFAULT_DRIVE);
	}

    public Command driveWith(String name, Command command, Supplier<SwerveState> state) {
        Command swerveDriveCommand = swerve.getCommandsBuilder().driveByDriversInputs(state);
        Command wantedCommand = command.deadlineFor(swerveDriveCommand);
        return asSubsystemCommand(wantedCommand, name);
    }

	public AimAssist getAA(RobotState state) {
		if (state == RobotState.ALIGN_REEF) {
            if (ALIGN_REEF)
			    return AimAssist.REEF;
            else
                return AimAssist.NONE;
		}
        if (state == RobotState.SCORE) {
            if (SCORE)
                return AimAssist.BRANCH;
            else
                return AimAssist.NONE;
        }
        if (state == RobotState.ALGAE_REMOVE) {
            if (ALGAE_REMOVE)
                return AimAssist.ALGAE_REMOVE;
            else
                return AimAssist.NONE;
        }
        if (state == RobotState.ALGAE_INTAKE) {
            if (ALGAE_INTAKE)
                return AimAssist.ALGAE_INTAKE;
            else
                return AimAssist.NONE;
        }
        return AimAssist.NONE;
	}

	public Command setState(RobotState state) {
		return switch (state) {
			case AUTO_PRE_NET -> driveToPreNet();
			case PROCESSOR_SCORE -> fullyProcessorScore();
			case INTAKE -> intakeWithAimAssist();
			case ALGAE_REMOVE -> algaeRemove();
			case ALIGN_REEF, ARM_PRE_SCORE, PRE_SCORE, SCORE_WITHOUT_RELEASE, SCORE, ALGAE_INTAKE, PRE_CLIMB ->
				driveWith(state.name(), superstructure.setState(state), () -> SwerveState.DEFAULT_DRIVE.withAimAssist(getAA(state)));
			case
				MANUAL_CLIMB,
				EXIT_CLIMB,
				STOP_CLIMB,
                ELEVATOR_OPENING,
				CLOSE_CLIMB,
				HOLD_ALGAE,
				CLIMB_WITH_LIMIT_SWITCH,
				CLIMB_WITHOUT_LIMIT_SWITCH,
				TRANSFER_ALGAE_TO_END_EFFECTOR,
				NET,
				PRE_NET,
				ALGAE_OUTTAKE_FROM_INTAKE,
				ALGAE_OUTTAKE_FROM_END_EFFECTOR,
				CORAL_OUTTAKE,
				STAY_IN_PLACE,
				DRIVE ->
				driveWith(state.name(), superstructure.setState(state));
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
			() -> new SequentialCommandGroup(
				autoScore(),
				setState(RobotState.HOLD_ALGAE).until(this::isReadyToCloseSuperstructure),
				algaeRemove()
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
		return new SequentialCommandGroup(driveToPreNet(), setState(RobotState.NET));
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
			RobotState.INTAKE
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


	private Command afterScore() {
		return new DeferredCommand(
			() -> new SequentialCommandGroup(
				(ScoringHelpers.targetScoreLevel == ScoreLevel.L4
					? driveWith("Soft close l4", superstructure.softCloseL4())
					: driveWith("pre score hold l2 l3", superstructure.preScore()).until(this::isReadyToCloseSuperstructure)),
				setState(RobotState.DRIVE)
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
			() -> new SequentialCommandGroup(driveWith("Soft close net", getSuperstructure().softCloseNet()), setState(RobotState.DRIVE)),
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
			case STAY_IN_PLACE, CORAL_OUTTAKE -> setState(RobotState.STAY_IN_PLACE);
			case
                INTAKE,
                DRIVE,
                ALIGN_REEF,
                ALGAE_OUTTAKE_FROM_END_EFFECTOR,
                PROCESSOR_SCORE,
                ALGAE_OUTTAKE_FROM_INTAKE,
                ELEVATOR_OPENING,
                ALGAE_INTAKE ->
				setState(RobotState.DRIVE);
			case AUTO_PRE_NET, PRE_NET, NET -> afterNet();
			case ALGAE_REMOVE, HOLD_ALGAE, TRANSFER_ALGAE_TO_END_EFFECTOR -> setState(RobotState.HOLD_ALGAE);
			case ARM_PRE_SCORE, CLOSE_CLIMB -> setState(RobotState.ARM_PRE_SCORE);
			case PRE_SCORE -> setState(RobotState.PRE_SCORE);
			case SCORE, SCORE_WITHOUT_RELEASE -> afterScore();
			case PRE_CLIMB -> setState(RobotState.PRE_CLIMB);
			case CLIMB_WITHOUT_LIMIT_SWITCH, CLIMB_WITH_LIMIT_SWITCH, MANUAL_CLIMB, EXIT_CLIMB, STOP_CLIMB -> setState(RobotState.STOP_CLIMB);
		};
	}

}
