package frc.robot.superstructers.poseestimator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.poseestimation.observations.OdometryObservation;
import frc.robot.poseestimation.poseestimator.PoseCalculator;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import org.littletonrobotics.junction.Logger;


public class PoseEstimatorSuperstructure {

    private final PoseCalculator poseCalculator;
    private final Swerve swerve;

    public PoseEstimatorSuperstructure(Swerve swerve) {
        this.poseCalculator = new PoseCalculator(PoseEstimatorConstants.ODOMETRY_STANDARD_DEVIATIONS, SwerveConstants.KINEMATICS);
        this.swerve = swerve;
        swerve.setCurrentAngleSupplier(() -> getCurrentPose().getRotation());
        resetPose(PoseEstimatorConstants.DEFAULT_POSE);
    }

    public Pose2d getCurrentPose(){
        return poseCalculator.getEstimatedPose();
    }

    public void resetPose(Pose2d currentPose) {
        swerve.setHeading(currentPose.getRotation());
        poseCalculator.resetPose(currentPose);
    }

    public void resetHeading(Rotation2d targetAngle) {
        resetPose(new Pose2d(getCurrentPose().getTranslation(), targetAngle));
    }

    private void logCurrentPose(){
        Logger.recordOutput(PoseEstimatorConstants.LOG_PATH + "Estimated Pose", getCurrentPose());
    }

    private void logCurrentOdometryPose(){
        Logger.recordOutput(PoseEstimatorConstants.LOG_PATH + "Odometry Pose", getCurrentPose());
    }


    /**
     * Updates the pose estimator with the given swerve wheel positions and gyro rotations.
     * This function accepts an array of swerve wheel positions and an array of gyro rotations because the odometry can be updated
     * at a faster rate than the main loop (which is 50 hertz).
     * This means you could have a couple of odometry updates per main loop, and you would want to update the pose estimator with
     * all of them.
     */
    public void updatePoseEstimatorOdometry(OdometryObservation[] odometryObservations) {
        for (OdometryObservation odometryObservation : odometryObservations) {
            poseCalculator.addOdometryObservation(odometryObservation);
        }
    }

    public void updatePoseEstimator(){
        swerve.updateInputs();
        updatePoseEstimatorOdometry(swerve.getAllOdometryObservations());

        logCurrentPose();
        logCurrentOdometryPose();
    }


    private static boolean isAtTranslationPosition(double currentTranslationVelocity, double currentTranslationPosition, double targetTranslationPosition) {
        boolean isNearTargetPosition = MathUtil.isNear(
                targetTranslationPosition,
                currentTranslationPosition,
                PoseEstimatorConstants.TRANSLATION_TOLERANCE_METERS
        );
        boolean isStopping = Math.abs(currentTranslationVelocity) < PoseEstimatorConstants.TRANSLATION_VELOCITY_TOLERANCE;
        return isNearTargetPosition && isStopping;
    }

    public boolean isAtXAxisPosition(double targetXBlueAlliancePosition) {
        return isAtTranslationPosition(
                swerve.getFieldRelativeVelocity().vxMetersPerSecond,
                getCurrentPose().getX(),
                targetXBlueAlliancePosition
        );
    }

    public boolean isAtYAxisPosition(double targetYBlueAlliancePosition) {
        return isAtTranslationPosition(
                swerve.getFieldRelativeVelocity().vyMetersPerSecond,
                getCurrentPose().getY(),
                targetYBlueAlliancePosition
        );
    }

    public boolean isAtAngle(Rotation2d targetAngle) {
        double angleDifferenceDeg = Math.abs(targetAngle.minus(getCurrentPose().getRotation()).getDegrees());
        boolean isAtAngle = angleDifferenceDeg < PoseEstimatorConstants.ROTATION_TOLERANCE.getDegrees();

        double currentRotationVelocityRadians = swerve.getRobotRelativeVelocity().omegaRadiansPerSecond;
        boolean isStopping = Math.abs(currentRotationVelocityRadians) < PoseEstimatorConstants.ROTATION_VELOCITY_TOLERANCE.getRadians();

        return isAtAngle && isStopping;
    }

    public boolean isAtPose(Pose2d targetBluePose) {
        return isAtXAxisPosition(targetBluePose.getX())
                && isAtYAxisPosition(targetBluePose.getY())
                && isAtAngle(targetBluePose.getRotation()
        );
    }

}
