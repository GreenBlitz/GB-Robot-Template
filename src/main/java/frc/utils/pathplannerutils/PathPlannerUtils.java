package frc.utils.pathplannerutils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.utils.mirrorutils.MirrorablePose2d;
import frc.utils.mirrorutils.MirrorableRotation2d;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;
import java.util.stream.Collectors;

public class PathPlannerUtils {

    public static void startPathPlanner() {
        makePathPlannerCompatibleWithAdvantageKit();
        scheduleWarmup();
    }

    private static void makePathPlannerCompatibleWithAdvantageKit() {
        Pathfinding.setPathfinder(new LocalADStarAK());
    }

    private static void scheduleWarmup() {
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
                resetPose,//todo - maybe cancel, if vision very accurate solving wrong robot placement
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

    public static void setDynamicObstacles(
            List<Pair<MirrorablePose2d, MirrorablePose2d>> obstacles,
            Pose2d currentRobotPose
    ) {
        List<Pair<Translation2d, Translation2d>> translationObstacles = obstacles.stream().map(pair ->
                new Pair<>(
                        pair.getFirst().get().getTranslation(),
                        pair.getSecond().get().getTranslation()
                )
        ).collect(Collectors.toList());
        Pathfinding.setDynamicObstacles(translationObstacles, currentRobotPose.getTranslation());
    }

    public static void setDynamicObstacles(Pair<MirrorablePose2d, MirrorablePose2d> obstacle, Pose2d currentRobotPose) {
        setDynamicObstacles(Collections.singletonList(obstacle), currentRobotPose);
    }

    public static void removeAllDynamicObstacles(Pose2d currentRobotPose) {
        List<Pair<MirrorablePose2d, MirrorablePose2d>> emptyList = new ArrayList<>();
        setDynamicObstacles(emptyList, currentRobotPose);
    }

    public static void setTargetRotationOverride(Supplier<Optional<MirrorableRotation2d>> overrider) {
        PPHolonomicDriveController.setRotationTargetOverride(() -> overrider.get().map(MirrorableRotation2d::get));
    }

    public static Command createOnTheFlyPathCommand(Pose2d currentBluePose, MirrorablePose2d targetPose,
            PathConstraints constraints
    ) {
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                currentBluePose,
                targetPose.get()
        );

        PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                constraints,
                new GoalEndState(0, targetPose.get().getRotation())
        );

        path.preventFlipping = true;

        return AutoBuilder.followPath(path);
    }

}
