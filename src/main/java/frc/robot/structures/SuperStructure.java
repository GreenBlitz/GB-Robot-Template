package frc.robot.structures;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.poseestimator.GBPoseEstimator;
import frc.robot.poseestimator.limelights.VisionObservationFiltered;
import frc.robot.poseestimator.observations.VisionObservation;
import frc.robot.subsystems.swerve.Swerve;

public class SuperStructure {

	private final Swerve swerve;
	private final GBPoseEstimator poseEstimator;
	private final VisionObservationFiltered visionObservationFiltered;

	public SuperStructure(Swerve swerve, GBPoseEstimator poseEstimator, VisionObservationFiltered visionObservationFiltered) {
		this.swerve = swerve;
		this.poseEstimator = poseEstimator;
		this.visionObservationFiltered = visionObservationFiltered;
	}

	public void periodic() {
		swerve.wrapperPeriodic();
		poseEstimator.updateOdometry(swerve.getAllOdometryObservations());
		for (VisionObservation visionObservation : visionObservationFiltered.getFilteredVisionObservations()) {
			poseEstimator.updateVision(visionObservation);
		}
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
			poseEstimator.getEstimatedPose().getX(),
			targetXBlueAlliancePosition
		);
	}

	public boolean isAtYAxisPosition(double targetYBlueAlliancePosition) {
		return isAtTranslationPosition(
			swerve.getFieldRelativeVelocity().vyMetersPerSecond,
			poseEstimator.getEstimatedPose().getY(),
			targetYBlueAlliancePosition
		);
	}

	public boolean isAtAngle(Rotation2d targetAngle) {
		double angleDifferenceDeg = Math.abs(targetAngle.minus(poseEstimator.getEstimatedPose().getRotation()).getDegrees());
		boolean isAtAngle = angleDifferenceDeg < SuperStructureConstants.HEADING_TOLERANCE.getDegrees();

		double currentRotationVelocityRadians = swerve.getRobotRelativeVelocity().omegaRadiansPerSecond;
		boolean isStopping = Math.abs(currentRotationVelocityRadians) < SuperStructureConstants.ROTATION_VELOCITY_TOLERANCE.getRadians();

		return isAtAngle && isStopping;
	}

	public boolean isAtPose(Pose2d targetBluePose) {
		return isAtXAxisPosition(targetBluePose.getX()) && isAtYAxisPosition(targetBluePose.getY()) && isAtAngle(targetBluePose.getRotation());
	}

}
