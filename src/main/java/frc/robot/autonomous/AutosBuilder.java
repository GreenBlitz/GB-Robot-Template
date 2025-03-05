package frc.robot.autonomous;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.constants.field.enums.Branch;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.ChassisPowers;
import frc.robot.scoringhelpers.ScoringHelpers;
import frc.robot.statemachine.superstructure.ScoreLevel;
import frc.utils.auto.AutoPath;
import frc.utils.auto.PathHelper;
import frc.utils.auto.PathPlannerAutoWrapper;
import frc.utils.auto.PathPlannerUtil;

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

	public static List<Supplier<PathPlannerAutoWrapper>> getAllPreBuiltAutos(
		Robot robot,
		Supplier<Command> intakingCommand,
		Supplier<Command> scoringCommand,
		Pose2d tolerance
	) {
		ArrayList<Supplier<PathPlannerAutoWrapper>> autos = new ArrayList<>();
		autos.add(() -> preBuiltLeftAuto(robot, intakingCommand, scoringCommand, tolerance));
		autos.add(() -> preBuiltCenterAuto(robot));
		autos.add(() -> preBuiltRightAuto(robot, intakingCommand, scoringCommand, tolerance));
		return autos;
	}

	public static List<Supplier<PathPlannerAutoWrapper>> getAllAutoScoringAutos(Robot robot) {
		ArrayList<Supplier<PathPlannerAutoWrapper>> autos = new ArrayList<>();
		for (Branch branch : Branch.values()) {
			autos.add(() -> autoScoreToBranch(branch, robot));
		}
		return autos;
	}

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

	public static PathPlannerAutoWrapper autoScoreToBranch(Branch branch, Robot robot) {
		return new PathPlannerAutoWrapper(new InstantCommand(() -> {
			ScoringHelpers.targetScoreLevel = ScoreLevel.L4;
			ScoringHelpers.isLeftBranch = branch.isLeft();
			ScoringHelpers.isFarReefHalf = branch.getReefSide().isFar();
			ScoringHelpers.setTargetSideForReef(branch.getReefSide().getSide());
		}).andThen(robot.getRobotCommander().autoScoreForAutonomous()), Pose2d.kZero, branch.name() + " Auto Score", true);
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

	private static PathPlannerAutoWrapper preBuiltRightAuto(
		Robot robot,
		Supplier<Command> intakingCommand,
		Supplier<Command> scoringCommand,
		Pose2d tolerance
	) {
		PathPlannerAutoWrapper auto = PathPlannerAutoWrapper.chainAutos(
			autoScoreToBranch(Branch.F, robot),
			PathPlannerAutoWrapper
				.chainAutos(
					createAutoFromAutoPath(
						AutoPath.F_TO_LOWER_CORAL_STATION_2,
						pathPlannerPath -> PathFollowingCommandsBuilder.deadlinePathWithCommand(
							robot,
							pathPlannerPath,
							intakingCommand,
							AutoPath.F_TO_LOWER_CORAL_STATION_2.getTargetBranch(),
							tolerance
						)
					),
					createAutoFromAutoPath(
						AutoPath.LOWER_CORAL_STATION_2_TO_C,
						pathPlannerPath -> PathFollowingCommandsBuilder.commandAfterPath(
							robot,
							pathPlannerPath,
							scoringCommand,
							AutoPath.LOWER_CORAL_STATION_2_TO_C.getTargetBranch(),
							tolerance
						)
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
					createAutoFromAutoPath(
						AutoPath.LOWER_CORAL_STATION_2_TO_D,
						pathPlannerPath -> PathFollowingCommandsBuilder.commandAfterPath(
							robot,
							pathPlannerPath,
							scoringCommand,
							AutoPath.LOWER_CORAL_STATION_2_TO_D.getTargetBranch(),
							tolerance
						)
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
					createAutoFromAutoPath(
						AutoPath.LOWER_CORAL_STATION_2_TO_E,
						pathPlannerPath -> PathFollowingCommandsBuilder.commandAfterPath(
							robot,
							pathPlannerPath,
							scoringCommand,
							AutoPath.LOWER_CORAL_STATION_2_TO_E.getTargetBranch(),
							tolerance
						)
					)
				)
				.asProxyAuto()
		);
		auto.setName("right");
		return auto;
	}

	private static PathPlannerAutoWrapper preBuiltCenterAuto(Robot robot) {
		PathPlannerAutoWrapper auto = autoScoreToBranch(Branch.H, robot);
		auto.setName("center");
		return auto;
	}

	private static PathPlannerAutoWrapper preBuiltLeftAuto(
		Robot robot,
		Supplier<Command> intakingCommand,
		Supplier<Command> scoringCommand,
		Pose2d tolerance
	) {
		PathPlannerAutoWrapper auto = PathPlannerAutoWrapper.chainAutos(
			autoScoreToBranch(Branch.I, robot),
			PathPlannerAutoWrapper
				.chainAutos(
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
					createAutoFromAutoPath(
						AutoPath.UPPER_CORAL_STATION_2_TO_L,
						pathPlannerPath -> PathFollowingCommandsBuilder.commandAfterPath(
							robot,
							pathPlannerPath,
							scoringCommand,
							AutoPath.UPPER_CORAL_STATION_2_TO_L.getTargetBranch(),
							tolerance
						)
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
					createAutoFromAutoPath(
						AutoPath.UPPER_CORAL_STATION_2_TO_K,
						pathPlannerPath -> PathFollowingCommandsBuilder.commandAfterPath(
							robot,
							pathPlannerPath,
							scoringCommand,
							AutoPath.UPPER_CORAL_STATION_2_TO_K.getTargetBranch(),
							tolerance
						)
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
					createAutoFromAutoPath(
						AutoPath.UPPER_CORAL_STATION_2_TO_J,
						pathPlannerPath -> PathFollowingCommandsBuilder.commandAfterPath(
							robot,
							pathPlannerPath,
							scoringCommand,
							AutoPath.UPPER_CORAL_STATION_2_TO_J.getTargetBranch(),
							tolerance
						)
					)
				)
				.asProxyAuto()
		);
		auto.setName("left");
		return auto;
	}

	public static PathPlannerAutoWrapper createDefaultAuto(Robot robot) {
		return new PathPlannerAutoWrapper(
			new ParallelCommandGroup(
				robot.getSwerve()
					.getCommandsBuilder()
					.drive(() -> new ChassisPowers(AutonomousConstants.DEFAULT_AUTO_DRIVE_POWER, 0, 0))
					.withTimeout(AutonomousConstants.DEFAULT_AUTO_DRIVE_TIME_SECONDS)
					.andThen(robot.getSwerve().getCommandsBuilder().resetTargetSpeeds()),
				robot.getRobotCommander().getSuperstructure().elevatorOpening()
			),
			Pose2d.kZero,
			"DefaultAuto",
			true
		);
	}

}
