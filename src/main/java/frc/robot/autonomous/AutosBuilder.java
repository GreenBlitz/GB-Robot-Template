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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.constants.field.Field;
import frc.constants.field.enums.Branch;
import frc.robot.Robot;
import frc.robot.statemachine.StateMachineConstants;
import frc.robot.statemachine.Tolerances;
import frc.robot.subsystems.elevator.ElevatorState;
import frc.robot.subsystems.swerve.ChassisPowers;
import frc.robot.scoringhelpers.ScoringHelpers;
import frc.robot.statemachine.superstructure.ScoreLevel;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.vision.DetectedObjectObseration;
import frc.utils.auto.PathHelper;
import frc.utils.auto.PathPlannerAutoWrapper;
import frc.utils.auto.PathPlannerUtil;
import frc.utils.math.AngleTransform;
import frc.utils.math.ToleranceMath;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

public class AutosBuilder {

	public static List<Supplier<PathPlannerAutoWrapper>> getAllTestAutos() {
		return List.of(
			() -> new PathPlannerAutoWrapper("Rotate"),
			() -> new PathPlannerAutoWrapper("Rotate 2m"),
			() -> new PathPlannerAutoWrapper("Straight 2m")
		);
	}

	public static List<Supplier<PathPlannerAutoWrapper>> getAllPreBuiltAutos(
		Robot robot,
		Supplier<Command> intakingCommand,
		Supplier<Command> scoringCommand
	) {
		ArrayList<Supplier<PathPlannerAutoWrapper>> autos = new ArrayList<>();
		autos.add(() -> leftAuto(robot, intakingCommand, scoringCommand));
		autos.add(() -> centerAuto(robot));
		autos.add(() -> rightAuto(robot, intakingCommand, scoringCommand));
		return autos;
	}

	public static List<Supplier<PathPlannerAutoWrapper>> getAllPreBuiltAutos(
		Robot robot,
		Supplier<Optional<DetectedObjectObseration>> algaeTranslationSupplier,
		Supplier<Command> intakingCommand,
		Supplier<Command> scoringCommand,
		Supplier<Command> algaeRemoveCommand,
		Supplier<Command> netCommand,
		Pose2d tolerance
	) {
		ArrayList<Supplier<PathPlannerAutoWrapper>> autos = new ArrayList<>();
		autos.add(() -> leftAuto(robot, intakingCommand, scoringCommand));
		autos.add(() -> centerAuto(robot));
		autos.add(() -> rightAuto(robot, intakingCommand, scoringCommand));
		autos.add(() -> autoBalls(robot, algaeRemoveCommand, netCommand, tolerance, Branch.G, ScoreLevel.L4));
		autos.add(() -> autoBalls(robot, algaeRemoveCommand, netCommand, tolerance, Branch.H, ScoreLevel.L4));
		autos.add(() -> autoBalls(robot, algaeRemoveCommand, netCommand, tolerance, Branch.G, ScoreLevel.L3));
		autos.add(() -> floorAutoBalls(robot, algaeTranslationSupplier, algaeRemoveCommand, netCommand, tolerance, Branch.G, ScoreLevel.L4));
		autos.add(() -> floorAutoBalls(robot, algaeTranslationSupplier, algaeRemoveCommand, netCommand, tolerance, Branch.H, ScoreLevel.L4));
		autos.add(() -> floorAutoBalls(robot, algaeTranslationSupplier, algaeRemoveCommand, netCommand, tolerance, Branch.G, ScoreLevel.L3));
		autos.add(() -> floorAutoBalls(robot, algaeTranslationSupplier, algaeRemoveCommand, netCommand, tolerance, Branch.H, ScoreLevel.L3));

		return autos;
	}

	public static List<Supplier<PathPlannerAutoWrapper>> getAllStartingAndScoringFirstObjectAutos(
		Swerve swerve,
		Supplier<Pose2d> currentPose,
		PathConstraints pathfindingConstraints,
		Supplier<Command> scoringCommand
	) {
		ArrayList<Supplier<PathPlannerAutoWrapper>> autos = new ArrayList<>();
		for (
			PathPlannerPath path : PathHelper.getAllStartingAndScoringFirstObjectPaths()
				.values()
				.stream()
				.sorted(Comparator.comparing(path -> path.name))
				.toList()
		) {
			autos
				.add(
					() -> new PathPlannerAutoWrapper(
						new InstantCommand(() -> ScoringHelpers.setTargetBranch(PathHelper.getPathTargetBranch(path))).alongWith(
							PathFollowingCommandsBuilder
								.deadlinePathWithCommand(swerve, currentPose, path, pathfindingConstraints, scoringCommand)
						),
						PathPlannerUtil.getPathStartingPose(path),
						path.name
					)
				);
		}
		return autos;
	}

	public static List<Supplier<PathPlannerAutoWrapper>> getAllAutoScoringAutos(Robot robot) {
		ArrayList<Supplier<PathPlannerAutoWrapper>> autos = new ArrayList<>();
		for (Branch branch : Branch.values()) {
			autos.add(() -> autoScoreToBranch(robot, branch, ScoreLevel.L4));
		}
		return autos;
	}

	public static List<Supplier<PathPlannerAutoWrapper>> getAllIntakingAutos(
		Swerve swerve,
		Supplier<Pose2d> currentPose,
		PathConstraints pathfindingConstraints,
		Supplier<Command> intakingCommand
	) {
		ArrayList<Supplier<PathPlannerAutoWrapper>> autos = new ArrayList<>();
		for (
			PathPlannerPath path : PathHelper.getAllIntakingPaths().values().stream().sorted(Comparator.comparing(path -> path.name)).toList()
		) {
			autos.add(
				() -> new PathPlannerAutoWrapper(
					PathFollowingCommandsBuilder.deadlinePathWithCommand(swerve, currentPose, path, pathfindingConstraints, intakingCommand),
					PathPlannerUtil.getPathStartingPose(path),
					path.name
				)
			);
		}
		return autos;
	}

	public static List<Supplier<PathPlannerAutoWrapper>> getAllScoringAutos(
		Swerve swerve,
		Supplier<Pose2d> currentPose,
		PathConstraints pathfindingConstraints,
		Supplier<Command> scoringCommand
	) {
		ArrayList<Supplier<PathPlannerAutoWrapper>> autos = new ArrayList<>();
		for (
			PathPlannerPath path : PathHelper.getAllScoringPathsFromCoralStations()
				.values()
				.stream()
				.sorted(Comparator.comparing(path -> path.name))
				.toList()
		) {
			autos
				.add(
					() -> new PathPlannerAutoWrapper(
						new InstantCommand(() -> ScoringHelpers.setTargetBranch(PathHelper.getPathTargetBranch(path))).alongWith(
							PathFollowingCommandsBuilder
								.deadlinePathWithCommand(swerve, currentPose, path, pathfindingConstraints, scoringCommand)
						),
						PathPlannerUtil.getPathStartingPose(path),
						path.name
					)
				);
		}
		return autos;
	}

	public static PathPlannerAutoWrapper createDefaultAuto(Robot robot) {
		ChassisPowers chassisPowers = new ChassisPowers();
		chassisPowers.xPower = AutonomousConstants.DEFAULT_AUTO_DRIVE_POWER;

		return new PathPlannerAutoWrapper(
			new ParallelCommandGroup(
				robot.getSwerve()
					.getCommandsBuilder()
					.drive(() -> chassisPowers)
					.withTimeout(AutonomousConstants.DEFAULT_AUTO_DRIVE_TIME_SECONDS)
					.andThen(robot.getSwerve().getCommandsBuilder().resetTargetSpeeds()),
				robot.getRobotCommander().getSuperstructure().elevatorOpening()
			),
			Pose2d.kZero,
			"Default Auto"
		);
	}

	public static PathPlannerAutoWrapper leftAuto(Robot robot, Supplier<Command> intakingCommand, Supplier<Command> scoringCommand) {
		return new PathPlannerAutoWrapper(
			new SequentialCommandGroup(
				autoScoreToBranch(robot, Branch.I, ScoreLevel.L4),
				new SequentialCommandGroup(
					PathFollowingCommandsBuilder.deadlinePathWithCommand(
						robot.getSwerve(),
						robot.getPoseEstimator()::getEstimatedPose,
						PathHelper.PATH_PLANNER_PATHS.get("I-US2"),
						AutonomousConstants.getRealTimeConstraintsForAuto(robot.getSwerve()),
						intakingCommand
					),
					new InstantCommand(() -> ScoringHelpers.setTargetBranch(Branch.L)).alongWith(
						PathFollowingCommandsBuilder.deadlinePathWithCommand(
							robot.getSwerve(),
							robot.getPoseEstimator()::getEstimatedPose,
							PathHelper.PATH_PLANNER_PATHS.get("US2-L"),
							AutonomousConstants.getRealTimeConstraintsForAuto(robot.getSwerve()),
							scoringCommand
						)
					),
					PathFollowingCommandsBuilder.deadlinePathWithCommand(
						robot.getSwerve(),
						robot.getPoseEstimator()::getEstimatedPose,
						PathHelper.PATH_PLANNER_PATHS.get("L-US2"),
						AutonomousConstants.getRealTimeConstraintsForAuto(robot.getSwerve()),
						intakingCommand
					),
					new InstantCommand(() -> ScoringHelpers.setTargetBranch(Branch.K)).alongWith(
						PathFollowingCommandsBuilder.deadlinePathWithCommand(
							robot.getSwerve(),
							robot.getPoseEstimator()::getEstimatedPose,
							PathHelper.PATH_PLANNER_PATHS.get("US2-K"),
							AutonomousConstants.getRealTimeConstraintsForAuto(robot.getSwerve()),
							scoringCommand
						)
					),
					PathFollowingCommandsBuilder.deadlinePathWithCommand(
						robot.getSwerve(),
						robot.getPoseEstimator()::getEstimatedPose,
						PathHelper.PATH_PLANNER_PATHS.get("K-US2"),
						AutonomousConstants.getRealTimeConstraintsForAuto(robot.getSwerve()),
						intakingCommand
					),
					new InstantCommand(() -> ScoringHelpers.setTargetBranch(Branch.A)).alongWith(
						PathFollowingCommandsBuilder.deadlinePathWithCommand(
							robot.getSwerve(),
							robot.getPoseEstimator()::getEstimatedPose,
							PathHelper.PATH_PLANNER_PATHS.get("US2-A"),
							AutonomousConstants.getRealTimeConstraintsForAuto(robot.getSwerve()),
							scoringCommand
						)
					)
				).asProxy()
			),
			Pose2d.kZero,
			"Left"
		);
	}

	private static PathPlannerAutoWrapper rightAuto(Robot robot, Supplier<Command> intakingCommand, Supplier<Command> scoringCommand) {
		return new PathPlannerAutoWrapper(
			new SequentialCommandGroup(
				autoScoreToBranch(robot, Branch.F, ScoreLevel.L4),
				new SequentialCommandGroup(
					PathFollowingCommandsBuilder.deadlinePathWithCommand(
						robot.getSwerve(),
						robot.getPoseEstimator()::getEstimatedPose,
						PathHelper.PATH_PLANNER_PATHS.get("F-LS2"),
						AutonomousConstants.getRealTimeConstraintsForAuto(robot.getSwerve()),
						intakingCommand
					),
					new InstantCommand(() -> ScoringHelpers.setTargetBranch(Branch.C)).alongWith(
						PathFollowingCommandsBuilder.deadlinePathWithCommand(
							robot.getSwerve(),
							robot.getPoseEstimator()::getEstimatedPose,
							PathHelper.PATH_PLANNER_PATHS.get("LS2-C"),
							AutonomousConstants.getRealTimeConstraintsForAuto(robot.getSwerve()),
							scoringCommand
						)
					),
					PathFollowingCommandsBuilder.deadlinePathWithCommand(
						robot.getSwerve(),
						robot.getPoseEstimator()::getEstimatedPose,
						PathHelper.PATH_PLANNER_PATHS.get("C-LS2"),
						AutonomousConstants.getRealTimeConstraintsForAuto(robot.getSwerve()),
						intakingCommand
					),
					new InstantCommand(() -> ScoringHelpers.setTargetBranch(Branch.D)).alongWith(
						PathFollowingCommandsBuilder.deadlinePathWithCommand(
							robot.getSwerve(),
							robot.getPoseEstimator()::getEstimatedPose,
							PathHelper.PATH_PLANNER_PATHS.get("LS2-D"),
							AutonomousConstants.getRealTimeConstraintsForAuto(robot.getSwerve()),
							scoringCommand
						)
					),
					PathFollowingCommandsBuilder.deadlinePathWithCommand(
						robot.getSwerve(),
						robot.getPoseEstimator()::getEstimatedPose,
						PathHelper.PATH_PLANNER_PATHS.get("D-LS2"),
						AutonomousConstants.getRealTimeConstraintsForAuto(robot.getSwerve()),
						intakingCommand
					),
					new InstantCommand(() -> ScoringHelpers.setTargetBranch(Branch.B)).alongWith(
						PathFollowingCommandsBuilder.deadlinePathWithCommand(
							robot.getSwerve(),
							robot.getPoseEstimator()::getEstimatedPose,
							PathHelper.PATH_PLANNER_PATHS.get("LS2-B"),
							AutonomousConstants.getRealTimeConstraintsForAuto(robot.getSwerve()),
							scoringCommand
						)
					)
				).asProxy()
			),
			Pose2d.kZero,
			"Right"
		);
	}

	private static PathPlannerAutoWrapper centerAuto(Robot robot) {
		return autoScoreToBranch(robot, Branch.H, ScoreLevel.L4).withAutoName("Center");
	}

	private static PathPlannerAutoWrapper autoBalls(
		Robot robot,
		Supplier<Command> algaeRemoveCommand,
		Supplier<Command> netCommand,
		Pose2d tolerance,
		Branch firstAutoScoreTargetBranch,
		ScoreLevel firstAutoScoreTargetScoreLevel
	) {
		Command autoBalls = new SequentialCommandGroup(
			autoScoreToBranch(robot, firstAutoScoreTargetBranch, firstAutoScoreTargetScoreLevel),
			new SequentialCommandGroup(
				getFirstAlgaeRemoveCommand(firstAutoScoreTargetScoreLevel, robot, firstAutoScoreTargetBranch, tolerance),
				PathFollowingCommandsBuilder.deadlinePathWithCommand(
					robot.getSwerve(),
					robot.getPoseEstimator()::getEstimatedPose,
					PathHelper.PATH_PLANNER_PATHS.get("ARD-LN"),
					AutonomousConstants.getRealTimeConstraintsForAuto(robot.getSwerve()),
					netCommand
				),
				new InstantCommand(() -> ScoringHelpers.setTargetBranch(Branch.I)),
				PathFollowingCommandsBuilder.deadlinePathWithCommand(
					robot.getSwerve(),
					robot.getPoseEstimator()::getEstimatedPose,
					PathHelper.PATH_PLANNER_PATHS.get("LN-ARE"),
					AutonomousConstants.getRealTimeConstraintsForAuto(robot.getSwerve()),
					algaeRemoveCommand
				),
				PathFollowingCommandsBuilder.deadlinePathWithCommand(
					robot.getSwerve(),
					robot.getPoseEstimator()::getEstimatedPose,
					PathHelper.PATH_PLANNER_PATHS.get("ARE-MN"),
					AutonomousConstants.getRealTimeConstraintsForAuto(robot.getSwerve()),
					netCommand
				),
				PathFollowingCommandsBuilder.deadlinePathWithCommand(
					robot.getSwerve(),
					robot.getPoseEstimator()::getEstimatedPose,
					PathHelper.PATH_PLANNER_PATHS.get("MN-ARC"),
					AutonomousConstants.getRealTimeConstraintsForAuto(robot.getSwerve()),
					algaeRemoveCommand
				),
				PathFollowingCommandsBuilder.deadlinePathWithCommand(
					robot.getSwerve(),
					robot.getPoseEstimator()::getEstimatedPose,
					PathHelper.PATH_PLANNER_PATHS.get("ARC-RN"),
					AutonomousConstants.getRealTimeConstraintsForAuto(robot.getSwerve()),
					netCommand
				)
			).asProxy()
		);

		String side = firstAutoScoreTargetBranch.isLeft() ? "left" : "right";

		return new PathPlannerAutoWrapper(autoBalls, Pose2d.kZero, side + " " + firstAutoScoreTargetScoreLevel.toString() + " auto balls");
	}

	private static PathPlannerAutoWrapper floorAutoBalls(
		Robot robot,
		Supplier<Optional<DetectedObjectObseration>> algaeTranslationSupplier,
		Supplier<Command> algaeRemoveCommand,
		Supplier<Command> netCommand,
		Pose2d tolerance,
		Branch firstAutoScoreTargetBranch,
		ScoreLevel firstAutoScoreTargetScoreLevel
	) {
		Pose2d backOffFromReefPose = Field.getAllianceRelative(
			Field.getReefSideMiddle(firstAutoScoreTargetBranch.getReefSide())
				.plus(new Transform2d(AutonomousConstants.BACK_OFF_FROM_REEF_DISTANCE_METERS, 0, new Rotation2d())),
			false,
			true,
			AngleTransform.MIRROR_Y
		);
		Supplier<Command> softCloseNet = () -> robot.getRobotCommander().getSuperstructure().softCloseNet().asProxy();

		Command bulbulBalls = new SequentialCommandGroup(
			autoScoreToBranch(robot, firstAutoScoreTargetBranch, firstAutoScoreTargetScoreLevel),
			new SequentialCommandGroup(
				getFirstAlgaeRemoveCommand(firstAutoScoreTargetScoreLevel, robot, firstAutoScoreTargetBranch, tolerance),
				PathFollowingCommandsBuilder.deadlinePathWithCommand(
					robot.getSwerve(),
					robot.getPoseEstimator()::getEstimatedPose,
					PathHelper.PATH_PLANNER_PATHS.get("ARD-LN"),
					AutonomousConstants.getRealTimeConstraintsForAuto(robot.getSwerve()),
					netCommand
				),
				PathFollowingCommandsBuilder.commandDuringPath(
					robot.getSwerve(),
					robot.getPoseEstimator()::getEstimatedPose,
					PathHelper.PATH_PLANNER_PATHS.get("LN-RFA"),
					AutonomousConstants.getRealTimeConstraintsForAuto(robot.getSwerve()),
					softCloseNet,
					tolerance
				),
				getFloorAlgaeToNetCommand(robot, algaeTranslationSupplier, algaeRemoveCommand, netCommand, tolerance, true),
				robot.getRobotCommander().getSuperstructure().netWithRelease().asProxy(),
				PathFollowingCommandsBuilder.commandDuringPath(
					robot.getSwerve(),
					robot.getPoseEstimator()::getEstimatedPose,
					PathHelper.PATH_PLANNER_PATHS.get("CLN-LFA"),
					AutonomousConstants.getRealTimeConstraintsForAuto(robot.getSwerve()),
					softCloseNet,
					tolerance
				),
				getFloorAlgaeToNetCommand(robot, algaeTranslationSupplier, algaeRemoveCommand, netCommand, tolerance, false),
				robot.getRobotCommander().getSuperstructure().netWithRelease().asProxy(),
				robot.getRobotCommander().getSuperstructure().idle().asProxy()
			).asProxy()
		);

		String side = firstAutoScoreTargetBranch.isLeft() ? "left" : "right";

		return new PathPlannerAutoWrapper(
			bulbulBalls,
			Pose2d.kZero,
			side + " " + firstAutoScoreTargetScoreLevel.toString() + " floor balls auto"
		);
	}

	public static PathPlannerAutoWrapper autoScoreToBranch(Robot robot, Branch branch, ScoreLevel scoreLevel) {
		return new PathPlannerAutoWrapper(
			robot.getRobotCommander()
				.autoScoreForAutonomous(getAutoScorePath(robot.getSwerve(), robot.getPoseEstimator()::getEstimatedPose, branch, scoreLevel)),
			Pose2d.kZero,
			branch.name() + " Auto Score"
		);
	}

	public static PathPlannerPath getAutoScorePath(Swerve swerve, Supplier<Pose2d> currentPose, Branch branch, ScoreLevel scoreLevel) {
		ScoringHelpers.targetScoreLevel = scoreLevel;
		ScoringHelpers.setTargetBranch(branch);
		Pose2d startingPose = currentPose.get();
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
			AutonomousConstants.getRealTimeConstraintsForAuto(swerve),
			new IdealStartingState(0, startingPose.getRotation()),
			new GoalEndState(0, scoringPose.getRotation()),
			false
		);
		path.preventFlipping = true;
		path.name = branch.name() + " Auto Score";
		Logger.recordOutput(AutonomousConstants.LOG_PATH_PREFIX + "/FirstPath", path.getPathPoses().toArray(Pose2d[]::new));
		return path;
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
		Supplier<Optional<DetectedObjectObseration>> algaeTranslationSupplier,
		Supplier<Command> algaeRemoveCommand,
		Supplier<Command> netCommand,
		Pose2d tolerance,
		boolean isRightFloorAlgae
	) {
		Supplier<Optional<Translation2d>> algaeTranslation = () -> algaeTranslationSupplier.get().isPresent()
			? Optional.of(
				algaeTranslationSupplier.get()
					.get()
					.robotRelativeObjectTranslation()
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
				? PathPlannerUtil.getPathStartingPose(PathHelper.PATH_PLANNER_PATHS.get("RFA-CLN"))
				: PathPlannerUtil.getPathStartingPose(PathHelper.PATH_PLANNER_PATHS.get("LFA-CRN")),
			true,
			true,
			AngleTransform.INVERT

		);
		Pose2d netLinkedWaypoint = Field.getAllianceRelative(
			isRightFloorAlgae
				? PathPlannerUtil.getLastPathPose(PathHelper.PATH_PLANNER_PATHS.get("RFA-CLN"))
				: PathPlannerUtil.getLastPathPose(PathHelper.PATH_PLANNER_PATHS.get("LFA-CRN")),
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

}
