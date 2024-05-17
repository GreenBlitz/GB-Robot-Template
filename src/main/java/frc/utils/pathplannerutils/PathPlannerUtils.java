package frc.utils.pathplannerutils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class PathPlannerUtils {

    public static void startPathPlanner() {
        makePathPlannerCompatibleWithAdvantageKit();
        scheduleWarmup();
    }

    public static void makePathPlannerCompatibleWithAdvantageKit() {
        Pathfinding.setPathfinder(new LocalADStarAK());
    }

    public static void scheduleWarmup() {
        PathfindingCommand.warmupCommand().schedule();
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
                resetPose,//todo - maybe cancel and base vision
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

    public static Command createOnTheFlyPathCommand(Pose2d currentPose, Pose2d targetPose, PathConstraints constraints) {
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                currentPose,
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
