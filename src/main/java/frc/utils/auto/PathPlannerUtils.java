package frc.utils.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GBSubsystem;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class PathPlannerUtils {

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
            PPHolonomicDriveController holonomicDriveController,
            RobotConfig robotConfig,
            BooleanSupplier shouldFlipPath,
            GBSubsystem... driveRequirements) {
        AutoBuilder.configure(
                poseSupplier,
                resetPose,
                robotRelativeSpeedsSupplier,
                robotRelativeSpeedsSetter,
                holonomicDriveController,
                robotConfig,
                shouldFlipPath,
                driveRequirements);
    }

    public static void registerCommand(String commandName, Command command) {
        NamedCommands.registerCommand(commandName, command);
    }

    public static void setDynamicObstacles(List<Pair<Translation2d, Translation2d>> obstacles, Pose2d currentPose) {
        dynamicObstacles = obstacles;
        Pathfinding.setDynamicObstacles(obstacles, currentPose.getTranslation());
    }

    public static void addDynamicObstacles(List<Pair<Translation2d, Translation2d>> obstacles, Pose2d currentPose) {
        List<Pair<Translation2d, Translation2d>> allObstacles = new ArrayList<>();
        allObstacles.addAll(dynamicObstacles);
        allObstacles.addAll(obstacles);
        setDynamicObstacles(allObstacles, currentPose);
    }

    public static void removeAllDynamicObstacles(Pose2d currentPose) {
        setDynamicObstacles(List.of(), currentPose);
    }

    public static Command createPathOnTheFly(Pose2d currentPose, Pose2d targetPose, PathConstraints constraints) {
        List<Waypoint> bezierPoints = PathPlannerPath.waypointsFromPoses(currentPose, targetPose);
        PathPlannerPath path =
                new PathPlannerPath(bezierPoints, constraints, null, new GoalEndState(0, targetPose.getRotation()));
        path.preventFlipping = true;
        return AutoBuilder.followPath(path);
    }
}
