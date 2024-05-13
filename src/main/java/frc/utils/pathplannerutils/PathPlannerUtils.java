package frc.utils.pathplannerutils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.utils.allianceutils.AlliancePose2d;
import frc.utils.allianceutils.AllianceRotation2d;
import frc.utils.allianceutils.AllianceTranslation2d;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
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

    public static void registerCommand(String commandName, Command command) {
        NamedCommands.registerCommand(commandName, command);
    }

    public static void setDynamicObstacles(
            List<Pair<AllianceTranslation2d, AllianceTranslation2d>> obstacles,
            AllianceTranslation2d currentRobotPose
    ) {
        List<Pair<Translation2d, Translation2d>> blueAllianceTranslationObstacles = obstacles.stream().map(
                pair -> new Pair<>(
                        pair.getFirst().getBlueAllianceTranslation2d(),
                        pair.getSecond().getBlueAllianceTranslation2d()
                )
        ).collect(Collectors.toList());
        Pathfinding.setDynamicObstacles(blueAllianceTranslationObstacles, currentRobotPose.getBlueAllianceTranslation2d());
    }

    public static void setDynamicObstacles(
            Pair<AllianceTranslation2d, AllianceTranslation2d> obstacle,
            AllianceTranslation2d currentRobotPose
    ) {
        setDynamicObstacles(Collections.singletonList(obstacle), currentRobotPose);
    }

    public static void removeAllDynamicObstacles(AllianceTranslation2d currentRobotPose) {
        List<Pair<AllianceTranslation2d, AllianceTranslation2d>> emptyList = new ArrayList<>();
        setDynamicObstacles(emptyList, currentRobotPose);
    }

    public static void setTargetRotationOverride(Supplier<Optional<AllianceRotation2d>> overrider) {
        PPHolonomicDriveController.setRotationTargetOverride(() -> overrider.get().map(AllianceRotation2d::getBlueAllianceAngle));
    }

    public static Command createOnTheFlyPathCommand(AlliancePose2d currentPose, AlliancePose2d targetPose,
            PathConstraints constraints) {
        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                currentPose.getBlueAlliancePose(),
                targetPose.getBlueAlliancePose()
        );

        PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                constraints,
                new GoalEndState(0, targetPose.getBlueAlliancePose().getRotation())
        );

        path.preventFlipping = true;

        return AutoBuilder.followPath(path);
    }

}
