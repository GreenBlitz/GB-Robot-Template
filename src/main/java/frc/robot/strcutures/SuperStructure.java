package frc.robot.strcutures;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.poseestimation.PoseEstimator;
import frc.robot.subsystems.swerve.Swerve;

public class SuperStructure {

	private final Swerve swerve;
	private final PoseEstimator poseEstimator;

	public SuperStructure(Swerve swerve, PoseEstimator poseEstimator) {
		this.swerve = swerve;
		this.poseEstimator = poseEstimator;
	}

	public void periodic() {
		swerve.wrapperPeriodic();
		poseEstimator.updatePoseEstimator(swerve.getAllOdometryObservations());
	}


	//@formatter:off
    private static boolean isAtTranslationPosition(double currentTranslationVelocity, double currentTranslationPosition, double targetTranslationPosition) {
        boolean isNearTargetPosition = MathUtil
                .isNear(targetTranslationPosition, currentTranslationPosition, SuperStructureConstants.TRANSLATION_TOLERANCE_METERS);
        boolean isStopping = Math.abs(currentTranslationVelocity) < SuperStructureConstants.TRANSLATION_VELOCITY_TOLERANCE;
        return isNearTargetPosition && isStopping;
    }
    //@formatter:on

	public boolean isAtXAxisPosition(double targetXBlueAlliancePosition) {
		return isAtTranslationPosition(
			swerve.getFieldRelativeVelocity().vxMetersPerSecond,
			poseEstimator.getCurrentPose().getX(),
			targetXBlueAlliancePosition
		);
	}

	public boolean isAtYAxisPosition(double targetYBlueAlliancePosition) {
		return isAtTranslationPosition(
			swerve.getFieldRelativeVelocity().vyMetersPerSecond,
			poseEstimator.getCurrentPose().getY(),
			targetYBlueAlliancePosition
		);
	}

	public boolean isAtAngle(Rotation2d targetAngle) {
		double angleDifferenceDeg = Math.abs(targetAngle.minus(poseEstimator.getCurrentPose().getRotation()).getDegrees());
		boolean isAtAngle = angleDifferenceDeg < SuperStructureConstants.ROTATION_TOLERANCE.getDegrees();

		double currentRotationVelocityRadians = swerve.getRobotRelativeVelocity().omegaRadiansPerSecond;
		boolean isStopping = Math.abs(currentRotationVelocityRadians) < SuperStructureConstants.ROTATION_VELOCITY_TOLERANCE.getRadians();

		return isAtAngle && isStopping;
	}

	public boolean isAtPose(Pose2d targetBluePose) {
		return isAtXAxisPosition(targetBluePose.getX()) && isAtYAxisPosition(targetBluePose.getY()) && isAtAngle(targetBluePose.getRotation());
	}

}
