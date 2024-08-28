package frc.utils.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class PathPlannerUtils {

    private static List<Pair<Translation2d, Translation2d>> dynamicObstacles = List.of();

    public static void startPathfinder() {
        setPathfinder();
        scheduleWarmup();
    }

    private static void setPathfinder() {
        Pathfinding.setPathfinder(new LocalADStar());
    }

    private static void scheduleWarmup() {
        PathfindingCommand.warmupCommand().schedule();
    }

    public static void setupLogging(String logPath) {
        PathPlannerLogging.setLogActivePathCallback(path -> Logger.recordOutput(logPath + "ActivePath", path.toArray(new Pose2d[0])));
        PathPlannerLogging.setLogTargetPoseCallback(targetPose -> Logger.recordOutput(logPath + "TargetPose", targetPose));
    }

    public static void configurePathPlanner(
            Supplier<Pose2d> poseSupplier,
            Consumer<Pose2d> resetPose,
            Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier,
            Consumer<ChassisSpeeds> robotRelativeOutput,
            HolonomicPathFollowerConfig config,
            BooleanSupplier shouldFlipPath,
            Subsystem driveSubsystem
    ) {
        AutoBuilder.configureHolonomic(
                poseSupplier,
                resetPose,
                robotRelativeSpeedsSupplier,
                robotRelativeOutput,
                config,
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
        dynamicObstacles.addAll(obstacles);
        setDynamicObstacles(dynamicObstacles, currentRobotPose);
    }

    public static void removeAllDynamicObstacles(Pose2d currentRobotPose) {
        setDynamicObstacles(List.of(), currentRobotPose);
    }

    public static void setTargetRotationOverride(Supplier<Optional<Rotation2d>> overrider) {
        PPHolonomicDriveController.setRotationTargetOverride(overrider);
    }

    public static Command createOnTheFlyPathCommand(Pose2d currentBluePose, Pose2d targetPose, PathConstraints constraints) {
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                currentBluePose,
                targetPose
        );

        PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                constraints,
                new GoalEndState(0, targetPose.getRotation())
        );

        path.preventFlipping = true;

        return AutoBuilder.followPath(path);
    }

}
