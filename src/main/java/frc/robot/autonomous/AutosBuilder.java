package frc.robot.autonomous;

import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.constants.field.Field;
import frc.constants.field.enums.Branch;
import frc.robot.Robot;
import frc.robot.statemachine.StateMachineConstants;
import frc.robot.statemachine.Tolerances;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.swerve.ChassisPowers;
import frc.robot.scoringhelpers.ScoringHelpers;
import frc.robot.statemachine.superstructure.ScoreLevel;
import frc.robot.vision.data.ObjectData;
import frc.utils.auto.AutoPath;
import frc.utils.auto.PathHelper;
import frc.utils.auto.PathPlannerAutoWrapper;
import frc.utils.auto.PathPlannerUtil;
import frc.utils.math.AngleTransform;
import frc.utils.math.ToleranceMath;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;
import java.util.function.Supplier;

public class AutosBuilder {

	public static List<Supplier<PathPlannerAutoWrapper>> getAllTestAutos() {
		return List.of(
			() -> new PathPlannerAutoWrapper("Rotate"),
			() -> new PathPlannerAutoWrapper("Rotate 2m"),
			() -> new PathPlannerAutoWrapper("Straight 2m")
		);
	}

	public static List<Supplier<PathPlannerAutoWrapper>> getAllStartingAndScoringFirstObjectAutos(
		Robot robot,
		Supplier<Command> scoringCommand,
		Pose2d tolerance
	) {
		ArrayList<Supplier<PathPlannerAutoWrapper>> autos = new ArrayList<>();
		for (AutoPath autoPath : PathHelper.getAllStartingAndScoringFirstObjectPaths()) {
			autos.add(
				() -> createAutoFromAutoPath(
					autoPath,
					pathPlannerPath -> PathFollowingCommandsBuilder
						.commandAfterPath(robot, pathPlannerPath, scoringCommand, autoPath.getTargetBranch(), tolerance)
				)
			);
		}
		return autos;
	}

//	public static List<Supplier<Command>> getAllPreBuiltAutos(
//		Robot robot,
//		Supplier<Command> intakingCommand,
//		Supplier<Command> scoringCommand,
//		Pose2d tolerance
//	) {
//		ArrayList<Supplier<Command>> autos = new ArrayList<>();
//		autos.add(() -> preBuiltLeftAuto(robot, intakingCommand, scoringCommand, tolerance));
//		autos.add(() -> preBuiltCenterAuto(robot));
//		autos.add(() -> preBuiltRightAuto(robot, intakingCommand, scoringCommand, tolerance));
//		return autos;
//	}

	public static List<Supplier<Command>> getAllNoDelayAutos(
		Robot robot,
		Supplier<Optional<ObjectData>> algaeTranslationSupplier,
		Supplier<Command> intakingCommand,
		Supplier<Command> scoringCommand,
		Supplier<Command> algaeRemoveCommand,
		Supplier<Command> netCommand,
		Pose2d tolerance
	) {
		ArrayList<Supplier<Command>> autos = new ArrayList<>();
		autos.add(() -> leftNoDelayAuto(robot, intakingCommand, scoringCommand, tolerance));
		autos.add(() -> centerNoDelayAuto(robot));
		autos.add(() -> rightNoDelayAuto(robot, intakingCommand, scoringCommand, tolerance));
		autos.add(() -> autoBalls(robot, algaeRemoveCommand, netCommand, tolerance, Branch.G, ScoreLevel.L4));
		autos.add(() -> autoBalls(robot, algaeRemoveCommand, netCommand, tolerance, Branch.H, ScoreLevel.L4));
		autos.add(() -> autoBalls(robot, algaeRemoveCommand, netCommand, tolerance, Branch.G, ScoreLevel.L3));
		autos.add(() -> floorAutoBalls(robot, algaeTranslationSupplier, algaeRemoveCommand, netCommand, tolerance, Branch.G, ScoreLevel.L4));
		autos.add(() -> floorAutoBalls(robot, algaeTranslationSupplier, algaeRemoveCommand, netCommand, tolerance, Branch.H, ScoreLevel.L4));
		autos.add(() -> floorAutoBalls(robot, algaeTranslationSupplier, algaeRemoveCommand, netCommand, tolerance, Branch.G, ScoreLevel.L3));
		autos.add(() -> floorAutoBalls(robot, algaeTranslationSupplier, algaeRemoveCommand, netCommand, tolerance, Branch.H, ScoreLevel.L3));

		return autos;
	}

//	public static List<Supplier<PathPlannerAutoWrapper>> getAllAutoScoringAutos(Robot robot) {
//		ArrayList<Supplier<PathPlannerAutoWrapper>> autos = new ArrayList<>();
//		for (Branch branch : Branch.values()) {
//			autos.add(() -> autoScoreToChosenBranch(branch, robot));
//		}
//		return autos;
//	}

	public static List<Supplier<PathPlannerAutoWrapper>> getAllIntakingAutos(Robot robot, Supplier<Command> intakingCommand, Pose2d tolerance) {
		ArrayList<Supplier<PathPlannerAutoWrapper>> autos = new ArrayList<>();
		for (AutoPath autoPath : PathHelper.getAllIntakingPaths()) {
			autos.add(
				() -> createAutoFromAutoPath(
					autoPath,
					pathPlannerPath -> PathFollowingCommandsBuilder
						.deadlinePathWithCommand(robot, pathPlannerPath, intakingCommand, autoPath.getTargetBranch(), tolerance)
				)
			);
		}
		return autos;
	}

	public static List<Supplier<PathPlannerAutoWrapper>> getAllScoringAutos(Robot robot, Supplier<Command> scoringCommand, Pose2d tolerance) {
		ArrayList<Supplier<PathPlannerAutoWrapper>> autos = new ArrayList<>();
		for (AutoPath autoPath : PathHelper.getAllScoringPathsFromCoralStations()) {
			autos.add(
				() -> createAutoFromAutoPath(
					autoPath,
					pathPlannerPath -> PathFollowingCommandsBuilder
						.commandAfterPath(robot, pathPlannerPath, scoringCommand, autoPath.getTargetBranch(), tolerance)
				)
			);
		}
		return autos;
	}

	public static PathPlannerAutoWrapper autoScoreToBranch(Branch branch, Robot robot, PathPlannerPath path) {
		return new PathPlannerAutoWrapper(new InstantCommand(() -> {
			ScoringHelpers.targetScoreLevel = ScoreLevel.L4;
			ScoringHelpers.isLeftBranch = branch.isLeft();
			ScoringHelpers.isFarReefHalf = branch.getReefSide().isFar();
			ScoringHelpers.setTargetSideForReef(branch.getReefSide().getSide());
		}).andThen(robot.getRobotCommander().autoScoreForAutonomous(path)), Pose2d.kZero, branch.name() + " Auto Score", true);
	}

	public static Command autoScoreToChosenBranch(Robot robot, PathPlannerPath path) {
		return robot.getRobotCommander().autoScoreForAutonomous(path);
	}

	public static PathPlannerPath getAutoScorePath(Branch branch, Robot robot, ScoreLevel scoreLevel) {
		ScoringHelpers.targetScoreLevel = scoreLevel;
		ScoringHelpers.setTargetBranch(branch);

		Pose2d startingPose = robot.getPoseEstimator().getEstimatedPose();
		Pose2d openSuperstructurePose = ScoringHelpers
			.getRobotBranchScoringPose(branch, StateMachineConstants.DISTANCE_TO_BRANCH_FOR_STARTING_PATH);
		Pose2d scoringPose = ScoringHelpers.getRobotBranchScoringPose(branch, StateMachineConstants.ROBOT_SCORING_DISTANCE_FROM_REEF_METERS);

		PathPlannerPath path = new PathPlannerPath(
			PathPlannerPath.waypointsFromPoses(startingPose, openSuperstructurePose, scoringPose),
			List.of(),
			List.of(),
			List.of(
				new ConstraintsZone(
					1,
					2,
					new PathConstraints(
						StateMachineConstants.MAX_VELOCITY_WHILE_ELEVATOR_L4_METERS_PER_SECOND,
						StateMachineConstants.MAX_ACCELERATION_WHILE_ELEVATOR_L4_METERS_PER_SECOND_SQUARED,
						StateMachineConstants.MAX_VELOCITY_WHILE_ELEVATOR_L4_ROTATION2D_PER_SECOND.getRadians(),
						StateMachineConstants.MAX_ACCELERATION_WHILE_ELEVATOR_L4_ROTATION2D_PER_SECOND_SQUARED.getRadians()
					)
				)
			),
			List.of(),
			AutonomousConstants.getRealTimeConstraintsForAuto(robot.getSwerve()),
			new IdealStartingState(0, startingPose.getRotation()),
			new GoalEndState(0, scoringPose.getRotation()),
			false
		);
		path.preventFlipping = true;
		Logger.recordOutput("Auto/FirstPath", path.getPathPoses().toArray(new Pose2d[] {}));
		return path;
	}

	public static PathPlannerAutoWrapper createAutoFromAutoPath(AutoPath path, Function<PathPlannerPath, Command> pathFollowingCommand) {
		Optional<PathPlannerPath> pathOptional = path.getPath();

		return new PathPlannerAutoWrapper(
			pathOptional.map(pathFollowingCommand).orElse(Commands.none()),
			pathOptional.map(PathPlannerUtil::getPathStartingPose).orElse(path.getStartingPoint().getSecond()),
			path.getPathName(),
			pathOptional.isPresent()
		);
	}

	public static Command leftNoDelayAuto(Robot robot, Supplier<Command> intakingCommand, Supplier<Command> scoringCommand, Pose2d tolerance) {
		PathPlannerPath path = getAutoScorePath(Branch.I, robot, ScoreLevel.L4);

		Command auto = new SequentialCommandGroup(
			autoScoreToChosenBranch(robot, path),
			new SequentialCommandGroup(
				createAutoFromAutoPath(
					AutoPath.I_TO_UPPER_CORAL_STATION_2,
					pathPlannerPath -> PathFollowingCommandsBuilder.deadlinePathWithCommand(
						robot,
						pathPlannerPath,
						intakingCommand,
						AutoPath.I_TO_UPPER_CORAL_STATION_2.getTargetBranch(),
						tolerance
					)
				),
				new InstantCommand(() -> ScoringHelpers.setTargetBranch(Branch.L)),
				createAutoFromAutoPath(
					AutoPath.UPPER_CORAL_STATION_2_TO_L,
					pathPlannerPath -> PathFollowingCommandsBuilder
						.scoreToBranch(robot, pathPlannerPath, scoringCommand, AutoPath.UPPER_CORAL_STATION_2_TO_L.getTargetBranch())
				),
				createAutoFromAutoPath(
					AutoPath.L_TO_UPPER_CORAL_STATION_2,
					pathPlannerPath -> PathFollowingCommandsBuilder.deadlinePathWithCommand(
						robot,
						pathPlannerPath,
						intakingCommand,
						AutoPath.L_TO_UPPER_CORAL_STATION_2.getTargetBranch(),
						tolerance
					)
				),
				new InstantCommand(() -> ScoringHelpers.setTargetBranch(Branch.K)),
				createAutoFromAutoPath(
					AutoPath.UPPER_CORAL_STATION_2_TO_K,
					pathPlannerPath -> PathFollowingCommandsBuilder
						.scoreToBranch(robot, pathPlannerPath, scoringCommand, AutoPath.UPPER_CORAL_STATION_2_TO_K.getTargetBranch())
				),
				createAutoFromAutoPath(
					AutoPath.K_TO_UPPER_CORAL_STATION_2,
					pathPlannerPath -> PathFollowingCommandsBuilder.deadlinePathWithCommand(
						robot,
						pathPlannerPath,
						intakingCommand,
						AutoPath.K_TO_UPPER_CORAL_STATION_2.getTargetBranch(),
						tolerance
					)
				),
				new InstantCommand(() -> ScoringHelpers.setTargetBranch(Branch.A)),
				createAutoFromAutoPath(
					AutoPath.UPPER_CORAL_STATION_2_TO_A,
					pathPlannerPath -> PathFollowingCommandsBuilder
						.scoreToBranch(robot, pathPlannerPath, scoringCommand, AutoPath.UPPER_CORAL_STATION_2_TO_A.getTargetBranch())
				)
			).asProxy()
		);
		auto.setName("left no delay");
		return auto;
	}

	private static Command rightNoDelayAuto(Robot robot, Supplier<Command> intakingCommand, Supplier<Command> scoringCommand, Pose2d tolerance) {
		PathPlannerPath path = getAutoScorePath(Branch.F, robot, ScoreLevel.L4);

		Command auto = new SequentialCommandGroup(
			autoScoreToChosenBranch(robot, path),
			new SequentialCommandGroup(
				createAutoFromAutoPath(
					AutoPath.F_TO_LOWER_CORAL_STATION_2,
					pathPlannerPath -> PathFollowingCommandsBuilder.deadlinePathWithCommand(
						robot,
						pathPlannerPath,
						intakingCommand,
						AutoPath.E_TO_LOWER_CORAL_STATION_2.getTargetBranch(),
						tolerance
					)
				),
				new InstantCommand(() -> ScoringHelpers.setTargetBranch(Branch.C)),
				createAutoFromAutoPath(
					AutoPath.LOWER_CORAL_STATION_2_TO_C,
					pathPlannerPath -> PathFollowingCommandsBuilder
						.scoreToBranch(robot, pathPlannerPath, scoringCommand, AutoPath.LOWER_CORAL_STATION_2_TO_C.getTargetBranch())
				),
				createAutoFromAutoPath(
					AutoPath.C_TO_LOWER_CORAL_STATION_2,
					pathPlannerPath -> PathFollowingCommandsBuilder.deadlinePathWithCommand(
						robot,
						pathPlannerPath,
						intakingCommand,
						AutoPath.C_TO_LOWER_CORAL_STATION_2.getTargetBranch(),
						tolerance
					)
				),
				new InstantCommand(() -> ScoringHelpers.setTargetBranch(Branch.D)),
				createAutoFromAutoPath(
					AutoPath.LOWER_CORAL_STATION_2_TO_D,
					pathPlannerPath -> PathFollowingCommandsBuilder
						.scoreToBranch(robot, pathPlannerPath, scoringCommand, AutoPath.LOWER_CORAL_STATION_2_TO_D.getTargetBranch())
				),
				createAutoFromAutoPath(
					AutoPath.D_TO_LOWER_CORAL_STATION_2,
					pathPlannerPath -> PathFollowingCommandsBuilder.deadlinePathWithCommand(
						robot,
						pathPlannerPath,
						intakingCommand,
						AutoPath.D_TO_LOWER_CORAL_STATION_2.getTargetBranch(),
						tolerance
					)
				),
				new InstantCommand(() -> ScoringHelpers.setTargetBranch(Branch.B)),
				createAutoFromAutoPath(
					AutoPath.LOWER_CORAL_STATION_2_TO_B,
					pathPlannerPath -> PathFollowingCommandsBuilder
						.scoreToBranch(robot, pathPlannerPath, scoringCommand, AutoPath.LOWER_CORAL_STATION_2_TO_B.getTargetBranch())
				)
			).asProxy()
		);
		auto.setName("right no delay");
		return auto;
	}

	private static Command getFirstAlgaeRemoveCommand(ScoreLevel scoreLevel, Robot robot, Branch firstAutoScoreTargetBranch, Pose2d tolerance) {
		boolean isL4 = scoreLevel == ScoreLevel.L4;
		Pose2d backOffFromReefPose;
		if (isL4) {
			backOffFromReefPose = Field.getAllianceRelative(
				Field.getReefSideMiddle(firstAutoScoreTargetBranch.getReefSide())
					.plus(new Transform2d(AutonomousConstants.BACK_OFF_FROM_REEF_DISTANCE_METERS, 0, new Rotation2d())),
				false,
				true,
				AngleTransform.MIRROR_Y
			);
		} else {
			backOffFromReefPose = Field.getAllianceRelative(
				Field.getReefSideMiddle(firstAutoScoreTargetBranch.getReefSide())
					.plus(new Transform2d(AutonomousConstants.BACK_OFF_FROM_REEF_DISTANCE_METERS, 0, new Rotation2d())),
				false,
				true,
				AngleTransform.MIRROR_Y
			);
		}
		Command algaeRemove = robot.getRobotCommander().getSuperstructure().algaeRemove().asProxy();
		Command afterScore = isL4
			? robot.getRobotCommander().getSuperstructure().holdAlgae().asProxy()
			: robot.getRobotCommander().getSuperstructure().stayInPlace().asProxy();
		ElevatorState elevatorStateAfterScore = isL4 ? ElevatorState.HOLD_ALGAE : ElevatorState.L3;
		return new SequentialCommandGroup(
			new ParallelCommandGroup(
				afterScore,
				robot.getSwerve().getCommandsBuilder().moveToPoseByPID(robot.getPoseEstimator()::getEstimatedPose, backOffFromReefPose)
			).until(
				() -> ToleranceMath.isNear(robot.getPoseEstimator().getEstimatedPose(), backOffFromReefPose, tolerance)
					&& robot.getElevator().isAtPosition(elevatorStateAfterScore.getHeightMeters(), Tolerances.ELEVATOR_HEIGHT_METERS)
			),
			new ParallelCommandGroup(
				algaeRemove,
				robot.getSwerve()
					.getCommandsBuilder()
					.moveToPoseByPID(robot.getPoseEstimator()::getEstimatedPose, ScoringHelpers.getAlgaeRemovePose(true))
			).withTimeout(AutonomousConstants.FIRST_ALGAE_REMOVE_TIMEOUT_SECONDS)
		);
	}

	private static Command getFloorAlgaeToNetCommand(
		Robot robot,
		Supplier<Optional<ObjectData>> algaeTranslationSupplier,
		Supplier<Command> algaeRemoveCommand,
		Supplier<Command> netCommand,
		Pose2d tolerance,
		boolean isRightFloorAlgae
	) {
		Supplier<Optional<Translation2d>> algaeTranslation = () -> algaeTranslationSupplier.get().isPresent()
			? Optional.of(
				algaeTranslationSupplier.get()
					.get()
					.getRobotRelativeEstimatedTranslation()
					.rotateBy(robot.getPoseEstimator().getEstimatedPose().getRotation())
					.plus(robot.getPoseEstimator().getEstimatedPose().getTranslation())
			)
			: Optional.of(
				isRightFloorAlgae
					? AutonomousConstants.DEFAULT_RIGHT_FLOOR_ALGAE_POSITION
					: AutonomousConstants.DEFAULT_LEFT_FLOOR_ALGAE_POSITION
			);
		Pose2d floorAlgaeLinkedWayPoint = Field.getAllianceRelative(
			isRightFloorAlgae
				? AutonomousConstants.LinkedWaypoints.RIGHT_FLOOR_ALGAE.getSecond()
				: AutonomousConstants.LinkedWaypoints.LEFT_FLOOR_ALGAE.getSecond(),
			true,
			true,
			AngleTransform.INVERT

		);
		Pose2d netLinkedWaypoint = Field.getAllianceRelative(
			isRightFloorAlgae
				? AutonomousConstants.LinkedWaypoints.CLOSE_LEFT_NET.getSecond()
				: AutonomousConstants.LinkedWaypoints.CLOSE_RIGHT_NET.getSecond(),
			true,
			true,
			AngleTransform.INVERT
		);
		Pose2d netLinkedWaypointMinus = new Pose2d(netLinkedWaypoint.getX() - 0.4, netLinkedWaypoint.getY(), netLinkedWaypoint.getRotation());
		return new SequentialCommandGroup(
			new ParallelCommandGroup(
				robot.getSwerve()
					.getCommandsBuilder()
					.driveToObject(
						() -> robot.getPoseEstimator().getEstimatedPose(),
						algaeTranslation,
						AutonomousConstants.DISTANCE_FROM_ALGAE_FOR_FLOOR_INTAKE,
						Rotation2d.fromDegrees(180)
					),
				robot.getRobotCommander().getSuperstructure().algaeIntake().asProxy()
			).until(() -> robot.getRobotCommander().getSuperstructure().isAlgaeInAlgaeIntake()),
			new ParallelDeadlineGroup(
				PathFollowingCommandsBuilder
					.pathfindToPose(netLinkedWaypointMinus, AutonomousConstants.getRealTimeConstraints(robot.getSwerve()))
					.andThen(
						PathFollowingCommandsBuilder
							.pathfindToPose(netLinkedWaypoint, AutonomousConstants.getRealTimeConstraints(robot.getSwerve()))
					),
				new SequentialCommandGroup(
					robot.getRobotCommander().getSuperstructure().transferAlgaeFromIntakeToEndEffector().asProxy(),
					robot.getRobotCommander().getSuperstructure().holdAlgae().asProxy().withTimeout(1),
					robot.getRobotCommander()
						.getSuperstructure()
						.preNet()
						.asProxy()
						.until(robot.getRobotCommander().getSuperstructure()::isPreNetReady)
				)
			),
			robot.getRobotCommander().getSuperstructure().netWithRelease().asProxy()

		);
	}

	private static Command floorAutoBalls(
		Robot robot,
		Supplier<Optional<ObjectData>> algaeTranslationSupplier,
		Supplier<Command> algaeRemoveCommand,
		Supplier<Command> netCommand,
		Pose2d tolerance,
		Branch firstAutoScoreTargetBranch,
		ScoreLevel firstAutoScoreTargetScoreLevel
	) {
		PathPlannerPath path = getAutoScorePath(firstAutoScoreTargetBranch, robot, firstAutoScoreTargetScoreLevel);
		Pose2d backOffFromReefPose = Field.getAllianceRelative(
			Field.getReefSideMiddle(firstAutoScoreTargetBranch.getReefSide())
				.plus(new Transform2d(AutonomousConstants.BACK_OFF_FROM_REEF_DISTANCE_METERS, 0, new Rotation2d())),
			false,
			true,
			AngleTransform.MIRROR_Y
		);
		ScoringHelpers.setTargetBranch(firstAutoScoreTargetBranch);
		Supplier<Command> softCloseNet = () -> robot.getRobotCommander().getSuperstructure().softCloseNet().asProxy();

		Command bulbulBalls = new SequentialCommandGroup(
			autoScoreToChosenBranch(robot, path),
			new SequentialCommandGroup(
				getFirstAlgaeRemoveCommand(firstAutoScoreTargetScoreLevel, robot, firstAutoScoreTargetBranch, tolerance),
				createAutoFromAutoPath(
					AutoPath.ALGAE_REMOVE_D_TO_LEFT_NET,
					pathPlannerPath -> PathFollowingCommandsBuilder
						.scoreToNet(robot, pathPlannerPath, netCommand, AutoPath.ALGAE_REMOVE_D_TO_LEFT_NET.getTargetBranch())
				),
				createAutoFromAutoPath(
					AutoPath.LEFT_NET_TO_RIGHT_FLOOR_ALGAE,
					pathPlannerPath -> PathFollowingCommandsBuilder.commandDuringPath(
						robot,
						pathPlannerPath,
						softCloseNet,
						AutoPath.LEFT_NET_TO_RIGHT_FLOOR_ALGAE.getTargetBranch(),
						tolerance
					)
				),
				getFloorAlgaeToNetCommand(robot, algaeTranslationSupplier, algaeRemoveCommand, netCommand, tolerance, true),
				robot.getRobotCommander().getSuperstructure().netWithRelease().asProxy(),
				createAutoFromAutoPath(
					AutoPath.CLOSE_LEFT_NET_TO_LEFT_FLOOR_ALGAE,
					pathPlannerPath -> PathFollowingCommandsBuilder.commandDuringPath(
						robot,
						pathPlannerPath,
						softCloseNet,
						AutoPath.CLOSE_LEFT_NET_TO_LEFT_FLOOR_ALGAE.getTargetBranch(),
						tolerance
					)
				),
				getFloorAlgaeToNetCommand(robot, algaeTranslationSupplier, algaeRemoveCommand, netCommand, tolerance, false),
				robot.getRobotCommander().getSuperstructure().netWithRelease().asProxy(),
				robot.getRobotCommander().getSuperstructure().idle().asProxy()
			).asProxy()
		);

		String side = firstAutoScoreTargetBranch.isLeft() ? "left" : "right";

		bulbulBalls.setName(side + " " + firstAutoScoreTargetScoreLevel.toString() + " floor balls auto");

		return bulbulBalls;
	}

	private static Command autoBalls(
		Robot robot,
		Supplier<Command> algaeRemoveCommand,
		Supplier<Command> netCommand,
		Pose2d tolerance,
		Branch firstAutoScoreTargetBranch,
		ScoreLevel firstAutoScoreTargetScoreLevel
	) {
		PathPlannerPath path = getAutoScorePath(firstAutoScoreTargetBranch, robot, firstAutoScoreTargetScoreLevel);
		ScoringHelpers.setTargetBranch(firstAutoScoreTargetBranch);

		Command autoBalls = new SequentialCommandGroup(
			autoScoreToChosenBranch(robot, path),
			new SequentialCommandGroup(
				getFirstAlgaeRemoveCommand(firstAutoScoreTargetScoreLevel, robot, firstAutoScoreTargetBranch, tolerance),
				createAutoFromAutoPath(
					AutoPath.ALGAE_REMOVE_D_TO_LEFT_NET,
					pathPlannerPath -> PathFollowingCommandsBuilder
						.scoreToNet(robot, pathPlannerPath, netCommand, AutoPath.ALGAE_REMOVE_D_TO_LEFT_NET.getTargetBranch())
				),
				new InstantCommand(() -> ScoringHelpers.setTargetBranch(Branch.I)),
				createAutoFromAutoPath(
					AutoPath.LEFT_NET_TO_ALGAE_REMOVE_E,
					pathPlannerPath -> PathFollowingCommandsBuilder.deadlinePathWithCommand(
						robot,
						pathPlannerPath,
						algaeRemoveCommand,
						AutoPath.LEFT_NET_TO_ALGAE_REMOVE_E.getTargetBranch(),
						tolerance
					)
				),
				createAutoFromAutoPath(
					AutoPath.ALGAE_REMOVE_E_TO_MIDDLE_NET,
					pathPlannerPath -> PathFollowingCommandsBuilder
						.scoreToNet(robot, pathPlannerPath, netCommand, AutoPath.ALGAE_REMOVE_E_TO_MIDDLE_NET.getTargetBranch())
				),
				createAutoFromAutoPath(
					AutoPath.MIDDLE_NET_TO_ALGAE_REMOVE_C,
					pathPlannerPath -> PathFollowingCommandsBuilder.deadlinePathWithCommand(
						robot,
						pathPlannerPath,
						algaeRemoveCommand,
						AutoPath.MIDDLE_NET_TO_ALGAE_REMOVE_C.getTargetBranch(),
						tolerance
					)
				),
				createAutoFromAutoPath(
					AutoPath.ALGAE_REMOVE_C_TO_RIGHT_NET,
					pathPlannerPath -> PathFollowingCommandsBuilder
						.scoreToNet(robot, pathPlannerPath, netCommand, AutoPath.ALGAE_REMOVE_C_TO_RIGHT_NET.getTargetBranch())
				)
			).asProxy()
		);

		String side = firstAutoScoreTargetBranch.isLeft() ? "left" : "right";

		autoBalls.setName(side + " " + firstAutoScoreTargetScoreLevel.toString() + " auto balls");

		return autoBalls;
	}

	private static Command centerNoDelayAuto(Robot robot) {
		PathPlannerPath path = getAutoScorePath(Branch.H, robot, ScoreLevel.L4);

		Command auto = autoScoreToChosenBranch(robot, path);
		auto.setName("center no delay");
		return auto;
	}


	public static Command createDefaultNoDelayAuto(Robot robot) {
		ChassisPowers chassisPowers = new ChassisPowers();
		chassisPowers.xPower = AutonomousConstants.DEFAULT_AUTO_DRIVE_POWER;

		Command auto = new ParallelCommandGroup(
			robot.getSwerve()
				.getCommandsBuilder()
				.drive(() -> chassisPowers)
				.withTimeout(AutonomousConstants.DEFAULT_AUTO_DRIVE_TIME_SECONDS)
				.andThen(robot.getSwerve().getCommandsBuilder().resetTargetSpeeds()),
			robot.getRobotCommander().getSuperstructure().elevatorOpening()
		);
		auto.setName("default no delay");
		return auto;
	}

//	private static PathPlannerAutoWrapper preBuiltRightAuto(
//		Robot robot,
//		Supplier<Command> intakingCommand,
//		Supplier<Command> scoringCommand,
//		Pose2d tolerance
//	) {
//		PathPlannerAutoWrapper auto = PathPlannerAutoWrapper.chainAutos(
//			autoScoreToChosenBranch(Branch.F, robot),
//			PathPlannerAutoWrapper
//				.chainAutos(
//					createAutoFromAutoPath(
//						AutoPath.F_TO_LOWER_CORAL_STATION_2,
//						pathPlannerPath -> PathFollowingCommandsBuilder.deadlinePathWithCommand(
//							robot,
//							pathPlannerPath,
//							intakingCommand,
//							AutoPath.F_TO_LOWER_CORAL_STATION_2.getTargetBranch(),
//							tolerance
//						)
//					),
//					createAutoFromAutoPath(
//						AutoPath.LOWER_CORAL_STATION_2_TO_C,
//						pathPlannerPath -> PathFollowingCommandsBuilder.commandAfterPath(
//							robot,
//							pathPlannerPath,
//							scoringCommand,
//							AutoPath.LOWER_CORAL_STATION_2_TO_C.getTargetBranch(),
//							tolerance
//						)
//					),
//					createAutoFromAutoPath(
//						AutoPath.C_TO_LOWER_CORAL_STATION_2,
//						pathPlannerPath -> PathFollowingCommandsBuilder.deadlinePathWithCommand(
//							robot,
//							pathPlannerPath,
//							intakingCommand,
//							AutoPath.C_TO_LOWER_CORAL_STATION_2.getTargetBranch(),
//							tolerance
//						)
//					),
//					createAutoFromAutoPath(
//						AutoPath.LOWER_CORAL_STATION_2_TO_D,
//						pathPlannerPath -> PathFollowingCommandsBuilder.commandAfterPath(
//							robot,
//							pathPlannerPath,
//							scoringCommand,
//							AutoPath.LOWER_CORAL_STATION_2_TO_D.getTargetBranch(),
//							tolerance
//						)
//					),
//					createAutoFromAutoPath(
//						AutoPath.D_TO_LOWER_CORAL_STATION_2,
//						pathPlannerPath -> PathFollowingCommandsBuilder.deadlinePathWithCommand(
//							robot,
//							pathPlannerPath,
//							intakingCommand,
//							AutoPath.D_TO_LOWER_CORAL_STATION_2.getTargetBranch(),
//							tolerance
//						)
//					),
//					createAutoFromAutoPath(
//						AutoPath.LOWER_CORAL_STATION_2_TO_E,
//						pathPlannerPath -> PathFollowingCommandsBuilder.commandAfterPath(
//							robot,
//							pathPlannerPath,
//							scoringCommand,
//							AutoPath.LOWER_CORAL_STATION_2_TO_E.getTargetBranch(),
//							tolerance
//						)
//					)
//				)
//				.asProxyAuto()
//		);
//		auto.setName("right");
//		return auto;
//	}
//
//	private static PathPlannerAutoWrapper preBuiltCenterAuto(Robot robot) {
//		PathPlannerAutoWrapper auto = autoScoreToChosenBranch(Branch.H, robot);
//		auto.setName("center");
//		return auto;
//	}
//
//	private static PathPlannerAutoWrapper preBuiltLeftAuto(
//		Robot robot,
//		Supplier<Command> intakingCommand,
//		Supplier<Command> scoringCommand,
//		Pose2d tolerance
//	) {
//		PathPlannerAutoWrapper auto = PathPlannerAutoWrapper.chainAutos(
//			autoScoreToChosenBranch(Branch.I, robot),
//			PathPlannerAutoWrapper
//				.chainAutos(
//					createAutoFromAutoPath(
//						AutoPath.I_TO_UPPER_CORAL_STATION_2,
//						pathPlannerPath -> PathFollowingCommandsBuilder.deadlinePathWithCommand(
//							robot,
//							pathPlannerPath,
//							intakingCommand,
//							AutoPath.I_TO_UPPER_CORAL_STATION_2.getTargetBranch(),
//							tolerance
//						)
//					),
//					createAutoFromAutoPath(
//						AutoPath.UPPER_CORAL_STATION_2_TO_L,
//						pathPlannerPath -> PathFollowingCommandsBuilder.commandAfterPath(
//							robot,
//							pathPlannerPath,
//							scoringCommand,
//							AutoPath.UPPER_CORAL_STATION_2_TO_L.getTargetBranch(),
//							tolerance
//						)
//					),
//					createAutoFromAutoPath(
//						AutoPath.L_TO_UPPER_CORAL_STATION_2,
//						pathPlannerPath -> PathFollowingCommandsBuilder.deadlinePathWithCommand(
//							robot,
//							pathPlannerPath,
//							intakingCommand,
//							AutoPath.L_TO_UPPER_CORAL_STATION_2.getTargetBranch(),
//							tolerance
//						)
//					),
//					createAutoFromAutoPath(
//						AutoPath.UPPER_CORAL_STATION_2_TO_K,
//						pathPlannerPath -> PathFollowingCommandsBuilder.commandAfterPath(
//							robot,
//							pathPlannerPath,
//							scoringCommand,
//							AutoPath.UPPER_CORAL_STATION_2_TO_K.getTargetBranch(),
//							tolerance
//						)
//					),
//					createAutoFromAutoPath(
//						AutoPath.K_TO_UPPER_CORAL_STATION_2,
//						pathPlannerPath -> PathFollowingCommandsBuilder.deadlinePathWithCommand(
//							robot,
//							pathPlannerPath,
//							intakingCommand,
//							AutoPath.K_TO_UPPER_CORAL_STATION_2.getTargetBranch(),
//							tolerance
//						)
//					),
//					createAutoFromAutoPath(
//						AutoPath.UPPER_CORAL_STATION_2_TO_J,
//						pathPlannerPath -> PathFollowingCommandsBuilder.commandAfterPath(
//							robot,
//							pathPlannerPath,
//							scoringCommand,
//							AutoPath.UPPER_CORAL_STATION_2_TO_J.getTargetBranch(),
//							tolerance
//						)
//					)
//				)
//				.asProxyAuto()
//		);
//		auto.setName("left");
//		return auto;
//	}
//
//	public static PathPlannerAutoWrapper createDefaultAuto(Robot robot) {
//		return new PathPlannerAutoWrapper(
//			new ParallelCommandGroup(
//				robot.getSwerve()
//					.getCommandsBuilder()
//					.drive(() -> new ChassisPowers(AutonomousConstants.DEFAULT_AUTO_DRIVE_POWER, 0, 0))
//					.withTimeout(AutonomousConstants.DEFAULT_AUTO_DRIVE_TIME_SECONDS)
//					.andThen(robot.getSwerve().getCommandsBuilder().resetTargetSpeeds()),
//				robot.getRobotCommander().getSuperstructure().elevatorOpening()
//			),
//			Pose2d.kZero,
//			"DefaultAuto",
//			true
//		);
//	}

}
