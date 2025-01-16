package frc.robot.autonomous;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.GBSubsystem;
import frc.utils.auto.PathPlannerUtils;
import frc.utils.math.ToleranceMath;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class RobotAutoHelper {

	public static void configPathPlanner(
		Supplier<Pose2d> poseSupplier,
		Consumer<Pose2d> resetPose,
		Supplier<ChassisSpeeds> robotRelativeSpeedsSupplier,
		Consumer<ChassisSpeeds> robotRelativeSpeedsSetter,
		PPHolonomicDriveController holonomicDriveController,
		RobotConfig robotConfig,
		BooleanSupplier shouldFlipPath,
		GBSubsystem... driveRequirements
	) {
		AutoBuilder.configure(
			poseSupplier,
			resetPose,
			robotRelativeSpeedsSupplier,
			robotRelativeSpeedsSetter,
			holonomicDriveController,
			robotConfig,
			shouldFlipPath,
			driveRequirements
		);
	}

	public static Command followPath(PathPlannerPath path) {
		return AutoBuilder.followPath(path);
	}

	public static Command pathfindToPose(Pose2d targetPose, PathConstraints pathfindingConstraints) {
		return AutoBuilder.pathfindToPose(targetPose, pathfindingConstraints);
	}

	public static Command pathfindThenFollowPath(PathPlannerPath path, PathConstraints pathfindingConstraints) {
		return AutoBuilder.pathfindThenFollowPath(path, pathfindingConstraints);
	}

	public static Command createPathOnTheFly(Pose2d currentPose, Pose2d targetPose, PathConstraints constraints) {
		List<Waypoint> bezierPoints = PathPlannerPath.waypointsFromPoses(currentPose, targetPose);
		PathPlannerPath path = new PathPlannerPath(bezierPoints, constraints, null, new GoalEndState(0, targetPose.getRotation()));
		path.preventFlipping = true;
		return followPath(path);
	}

	public static Command followPathOrDriveToPathEnd(Robot robot, PathPlannerPath path) {
		return robot.getSwerve()
			.getCommandsBuilder()
			.followPathOrDriveToPathEnd(robot.getPoseEstimator()::getCurrentPose, path)
			.until(
				() -> ToleranceMath.isNear(
					getAllianceRelativePose(PathPlannerUtils.getLastPathPose(path)),
					robot.getPoseEstimator().getCurrentPose(),
					AutonomousConstants.TARGET_ANGLE_TOLERANCE,
					AutonomousConstants.DISTANCE_FROM_TARGET_TOLERANCE_METERS
				)
			);
	}

	public static boolean isRobotCloseToPathBeginning(PathPlannerPath path, Supplier<Pose2d> currentPose, double toleranceMeters) {
		return ToleranceMath
			.isNear(getAllianceRelativePose(path.getPathPoses().get(0)).getTranslation(), currentPose.get().getTranslation(), toleranceMeters);
	}

	public static Pose2d getAllianceRelativePose(Pose2d bluePose) {
		return AutoBuilder.shouldFlip() ? FlippingUtil.flipFieldPose(bluePose) : bluePose;
	}

}
