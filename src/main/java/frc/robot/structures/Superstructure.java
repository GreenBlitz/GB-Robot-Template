package frc.robot.structures;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.constants.RobotHeadingEstimatorConstants;
import frc.robot.poseestimator.IPoseEstimator;
import frc.robot.poseestimator.helpers.RobotHeadingEstimator;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.vision.multivisionsources.MultiAprilTagVisionSources;
import frc.utils.TimedValue;
import frc.utils.time.TimeUtils;
import org.littletonrobotics.junction.Logger;

public class Superstructure {

	private final Swerve swerve;
	private final IPoseEstimator poseEstimator;
	private final RobotHeadingEstimator headingEstimator;
	private final MultiAprilTagVisionSources multiAprilTagVisionSources;

	public Superstructure(
		Swerve swerve,
		IPoseEstimator poseEstimator,
		RobotHeadingEstimator headingEstimator,
		MultiAprilTagVisionSources multiAprilTagVisionSources
	) {
		this.swerve = swerve;
		this.poseEstimator = poseEstimator;
		this.headingEstimator = headingEstimator;
		this.multiAprilTagVisionSources = multiAprilTagVisionSources;
	}

	public void periodic() {
		swerve.update();
		Logger.recordOutput("GyroValue", swerve.getGyroAbsoluteYaw().getDegrees());
		headingEstimator.updateGyroAngle(new TimedValue<>(swerve.getGyroAbsoluteYaw(), TimeUtils.getCurrentTimeSeconds()));
		for(TimedValue<Rotation2d> heading : multiAprilTagVisionSources.getFilteredRobotHeading()) {
			headingEstimator.updateVisionIfNotCalibrated(
				heading,
				RobotHeadingEstimatorConstants.DEFAULT_VISION_STANDARD_DEVIATION,
				RobotHeadingEstimatorConstants.MAXIMUM_STANDARD_DEVIATION_TOLERANCE
			);
		}
		headingEstimator.log();
		poseEstimator.updateOdometry(swerve.getAllOdometryData());
		poseEstimator.updateVision(multiAprilTagVisionSources.getFilteredVisionData());
	}


	private static boolean isAtTranslationPosition(
		double currentTranslationVelocity,
		double currentTranslationPosition,
		double targetTranslationPosition
	) {
		boolean isNearTargetPosition = MathUtil.isNear(targetTranslationPosition, currentTranslationPosition, Tolerances.TRANSLATION_METERS);
		boolean isStopping = Math.abs(currentTranslationVelocity) < Tolerances.TRANSLATION_VELOCITY_DEADBAND;
		return isNearTargetPosition && isStopping;
	}

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
		boolean isAtAngle = angleDifferenceDeg < Tolerances.SWERVE_HEADING.getDegrees();

		double currentRotationVelocityRadians = swerve.getRobotRelativeVelocity().omegaRadiansPerSecond;
		boolean isStopping = Math.abs(currentRotationVelocityRadians) < Tolerances.ROTATION_VELOCITY_DEADBAND.getRadians();

		return isAtAngle && isStopping;
	}

	public boolean isAtPose(Pose2d targetBluePose) {
		return isAtXAxisPosition(targetBluePose.getX()) && isAtYAxisPosition(targetBluePose.getY()) && isAtAngle(targetBluePose.getRotation());
	}

}
