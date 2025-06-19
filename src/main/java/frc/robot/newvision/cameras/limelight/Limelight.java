package frc.robot.newvision.cameras.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.newvision.RobotPoseObservation;
import frc.robot.newvision.interfaces.OrientationRequiringRobotPoseSupplier;
import frc.robot.newvision.interfaces.IndependentRobotPoseSupplier;
import frc.utils.LimelightHelpers;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

public class Limelight implements IndependentRobotPoseSupplier, OrientationRequiringRobotPoseSupplier {

	private final String name;
	private final String logPath;
	private final Pose3d robotRelativeCameraPose;

	private final double minStandardDeviation;
	private final double translationStandardDeviationFactor;
	private final double rotationStandardDeviationFactor;

	private final RobotPoseObservation megaTag1RobotPoseObservation;
	private final RobotPoseObservation megaTag2RobotPoseObservation;

	private LimelightPipeline pipeline;

	public Limelight(
		String name,
		String logPathPrefix,
		Pose3d robotRelativeCameraPose,
		LimelightPipeline pipeline,
		double minStandardDeviation,
		double translationStandardDeviationFactor,
		double rotationStandardDeviationFactor
	) {
		this.name = name;
		this.logPath = logPathPrefix + "/" + name;

		this.robotRelativeCameraPose = robotRelativeCameraPose;
		setRobotRelativeCameraPose(robotRelativeCameraPose);

		this.minStandardDeviation = minStandardDeviation;
		this.translationStandardDeviationFactor = translationStandardDeviationFactor;
		this.rotationStandardDeviationFactor = rotationStandardDeviationFactor;

		this.megaTag1RobotPoseObservation = new RobotPoseObservation();
		this.megaTag2RobotPoseObservation = new RobotPoseObservation();

		setPipeline(pipeline);
	}

	public void update() {
		switch (pipeline) {
			case APRIL_TAG -> {
				LimelightHelpers.PoseEstimate megaTag1RobotPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
				LimelightHelpers.PoseEstimate megaTag2RobotPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

				megaTag1RobotPoseObservation.setObservationValues(
					megaTag1RobotPoseEstimate.timestampSeconds,
					megaTag1RobotPoseEstimate.pose,
					calculateStandardDeviations(megaTag1RobotPoseEstimate)
				);
				megaTag2RobotPoseObservation.setObservationValues(
					megaTag2RobotPoseEstimate.timestampSeconds,
					megaTag2RobotPoseEstimate.pose,
					calculateStandardDeviations(megaTag2RobotPoseEstimate)
				);
			}
			default -> {}
		}

		log();
	}

	private void log() {
		switch (pipeline) {
			case APRIL_TAG -> {
				Logger.recordOutput(logPath + "/megaTag1Pose", megaTag1RobotPoseObservation.getRobotPose());
				Logger.recordOutput(logPath + "/megaTag1StandardDeviations", megaTag1RobotPoseObservation.getStandardDeviations());
				Logger.recordOutput(logPath + "/megaTag2Pose", megaTag2RobotPoseObservation.getRobotPose());
				Logger.recordOutput(logPath + "/megaTag2StandardDeviations", megaTag2RobotPoseObservation.getStandardDeviations());
			}
			default -> {}
		}
	}

	public String getName() {
		return name;
	}

	public Pose3d getRobotRelativeCameraPose() {
		return robotRelativeCameraPose;
	}

	@Override
	public Optional<RobotPoseObservation> getRobotPose() {
		if (pipeline.equals(LimelightPipeline.APRIL_TAG)) {
			return Optional.of(megaTag1RobotPoseObservation);
		}
		return Optional.empty();
	}

	@Override
	public Optional<RobotPoseObservation> getOrientationRequiringRobotPose() {
		if (pipeline.equals(LimelightPipeline.APRIL_TAG)) {
			return Optional.of(megaTag2RobotPoseObservation);
		}
		return Optional.empty();
	}

	@Override
	public void setRobotOrientation(Rotation3d robotOrientation) {
		LimelightHelpers.SetRobotOrientation(
			name,
			Math.toDegrees(robotOrientation.getZ()),
			0,
			Math.toDegrees(robotOrientation.getY()),
			0,
			Math.toDegrees(robotOrientation.getX()),
			0
		);
	}

	public void setPipeline(LimelightPipeline pipeline) {
		this.pipeline = pipeline;
		LimelightHelpers.setPipelineIndex(name, pipeline.getPipelineIndex());
	}

	private void setRobotRelativeCameraPose(Pose3d robotRelativeCameraPose) {
		LimelightHelpers.setCameraPose_RobotSpace(
			name,
			robotRelativeCameraPose.getX(),
			robotRelativeCameraPose.getY(),
			robotRelativeCameraPose.getZ(),
			robotRelativeCameraPose.getRotation().getX(),
			robotRelativeCameraPose.getRotation().getY(),
			robotRelativeCameraPose.getRotation().getZ()
		);
	}

	private Pose2d calculateStandardDeviations(LimelightHelpers.PoseEstimate poseEstimate) {
		double translationStandardDeviation = Math
			.max(Math.pow(poseEstimate.avgTagDist, 2) * translationStandardDeviationFactor, minStandardDeviation);
		double rotationStandardDeviation = Math
			.max(Math.pow(poseEstimate.avgTagDist, 2) * rotationStandardDeviationFactor, minStandardDeviation);
		return new Pose2d(translationStandardDeviation, translationStandardDeviation, new Rotation2d(rotationStandardDeviation));
	}

}
