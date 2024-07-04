// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.poseestimation.poseestimator.PoseEstimator;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveState;
import frc.utils.DriverStationUtils;
import frc.utils.RobotTypeUtils;
import frc.utils.pathplannerutils.PathPlannerUtils;
import org.littletonrobotics.junction.Logger;


public class Robot {

    public static final RobotTypeUtils.RobotType ROBOT_TYPE = RobotTypeUtils.determineRobotType(RobotTypeUtils.RobotType.REAL);

    public static final Swerve swerve = new Swerve(); // Must be before POSE_ESTIMATOR
    private static final PoseEstimator poseEstimator = new PoseEstimator(RobotConstants.DEFAULT_POSE);

    public Robot() {
        buildPathPlannerForAuto();
        configureBindings();
    }

    private void buildPathPlannerForAuto() {
        // Register commands...
        PathPlannerUtils.configurePathPlanner(
                Robot::getCurrentPose,
                Robot::resetPose,
                Robot.swerve::getSelfRelativeVelocity,
                (speeds) -> Robot.swerve.driveByState(speeds, SwerveState.DEFAULT_PATH_PLANNER),
                SwerveConstants.HOLONOMIC_PATH_FOLLOWER_CONFIG,
                DriverStationUtils::isRedAlliance,
                Robot.swerve
        );
    }

    private void configureBindings() {
        JoysticksBindings.configureBindings();
    }

    public Command getAutonomousCommand() {
        return new InstantCommand();
    }


    private static boolean isAtTranslationPosition(double currentTranslationVelocity, double currentTranslationPosition, double targetTranslationPosition) {
        boolean isNearTargetPosition = MathUtil.isNear(
                targetTranslationPosition,
                currentTranslationPosition,
                RobotConstants.TRANSLATION_TOLERANCE_METERS
        );
        boolean isStopping = Math.abs(currentTranslationVelocity) < RobotConstants.TRANSLATION_VELOCITY_TOLERANCE;
        return isNearTargetPosition && isStopping;
    }

    public static boolean isAtXAxisPosition(double targetXBlueAlliancePosition) {
        return isAtTranslationPosition(
                swerve.getFieldRelativeVelocity().vxMetersPerSecond,
                getCurrentPose().getX(),
                targetXBlueAlliancePosition
        );
    }

    public static boolean isAtYAxisPosition(double targetYBlueAlliancePosition) {
        return isAtTranslationPosition(
                swerve.getFieldRelativeVelocity().vyMetersPerSecond,
                getCurrentPose().getY(),
                targetYBlueAlliancePosition
        );
    }

    public static boolean isAtAngle(Rotation2d targetAngle) {
        double angleDifferenceDeg = Math.abs(targetAngle.minus(getCurrentPose().getRotation()).getDegrees());
        boolean isAtAngle = angleDifferenceDeg < RobotConstants.ROTATION_TOLERANCE.getDegrees();

        double currentRotationVelocityRadians = swerve.getSelfRelativeVelocity().omegaRadiansPerSecond;
        boolean isStopping = Math.abs(currentRotationVelocityRadians) < RobotConstants.ROTATION_VELOCITY_TOLERANCE.getRadians();

        return isAtAngle && isStopping;
    }

    public static boolean isAtPosition(Pose2d targetBluePose) {
        return isAtXAxisPosition(targetBluePose.getX())
                && isAtYAxisPosition(targetBluePose.getY())
                && isAtAngle(targetBluePose.getRotation()
        );
    }

    public static void resetPose(Pose2d currentPose) {
        swerve.setHeading(currentPose.getRotation());
        poseEstimator.resetPose(currentPose);
    }

    public static void resetHeading(Rotation2d targetAngle) {
        resetPose(new Pose2d(Robot.getCurrentPose().getTranslation(), targetAngle));
    }

    public static void updatePoseEstimator(){
        swerve.updateInputs();
        poseEstimator.updatePoseEstimatorOdometry(swerve.getAllOdometryObservations());
        Logger.recordOutput(frc.robot.RobotConstants.POSE_LOG_PATH, getCurrentPose());
    }

    public static Pose2d getCurrentPose(){
        return poseEstimator.poseEstimator6328.getEstimatedPose();
    }

}
