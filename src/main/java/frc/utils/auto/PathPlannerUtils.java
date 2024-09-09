package frc.utils.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class PathPlannerUtils {

	private static final boolean PREVENT_PATH_FLIPPING = true;

	private static List<Pair<Translation2d, Translation2d>> dynamicObstacles = List.of();

	public static void startPathfinder() {
		setPathfinder(new LocalADStar());
		scheduleWarmup();
	}

	public static void setPathfinder(Pathfinder pathfinder) {
		Pathfinding.setPathfinder(pathfinder);
	}

	public static void scheduleWarmup() {
		PathfindingCommand.warmupCommand().schedule();
	}

	public static void configPathPlanner(
		Supplier<Pose2d> poseSupplier,
		Consumer<Pose2d> resetPose,
		Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier,
		Consumer<ChassisSpeeds> robotRelativeSpeedsSetter,
		HolonomicPathFollowerConfig holonomicPathFollowerConfig,
		BooleanSupplier shouldFlipPath,
		Subsystem driveSubsystem
	) {
		AutoBuilder.configureHolonomic(
			poseSupplier,
			resetPose,
			robotRelativeSpeedsSupplier,
			robotRelativeSpeedsSetter,
			holonomicPathFollowerConfig,
			shouldFlipPath,
			driveSubsystem
		);
	}

	public static void registerCommand(String commandName, Command command) {
		NamedCommands.registerCommand(commandName, command);
	}

	public static void setDynamicObstacles(List<Pair<Translation2d, Translation2d>> obstacles, Pose2d currentRobotPose) {
		dynamicObstacles = obstacles;
		Pathfinding.setDynamicObstacles(obstacles, currentRobotPose.getTranslation());
	}

	public static void addDynamicObstacles(List<Pair<Translation2d, Translation2d>> obstacles, Pose2d currentRobotPose) {
		List<Pair<Translation2d, Translation2d>> allObstacles = new ArrayList<>();
		allObstacles.addAll(dynamicObstacles);
		allObstacles.addAll(obstacles);
		setDynamicObstacles(allObstacles, currentRobotPose);
	}

	public static void removeAllDynamicObstacles(Pose2d currentRobotPose) {
		setDynamicObstacles(List.of(), currentRobotPose);
	}

	public static void setRotationTargetOverride(Supplier<Optional<Rotation2d>> overrider) {
		PPHolonomicDriveController.setRotationTargetOverride(overrider);
	}

	// @formatter:off
	public static Command createPathOnTheFly(Pose2d currentPose, Pose2d targetPose, PathConstraints constraints, boolean preventFlipping) {
	// @formatter:on
		List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(currentPose, targetPose);
		PathPlannerPath path = new PathPlannerPath(bezierPoints, constraints, new GoalEndState(0, targetPose.getRotation()));
		path.preventFlipping = preventFlipping;
		return AutoBuilder.followPath(path);
	}

	public static Command createPathOnTheFly(Pose2d currentRobotPose, Pose2d targetPose, PathConstraints constraints) {
		return createPathOnTheFly(currentRobotPose, targetPose, constraints, PREVENT_PATH_FLIPPING);
	}

}
