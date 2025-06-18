package frc.robot.newvision.cameras.limelight;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.newvision.RobotPoseObservation;
import frc.robot.newvision.interfaces.RobotOrientationRequiringCamera;
import frc.robot.newvision.interfaces.IndependentRobotPoseSupplyingCamera;
import frc.utils.LimelightHelpers;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

public class Limelight implements IndependentRobotPoseSupplyingCamera, RobotOrientationRequiringCamera {

	private final String name;
	private final String logPath;
	private final Pose3d robotRelativeCameraPose;

	private final RobotPoseObservation megaTag1RobotPoseObservation;
	private final RobotPoseObservation megaTag2RobotPoseObservation;

	private LimelightPipeline pipeline;

	public Limelight(String name, String logPathPrefix, Pose3d robotRelativeCameraPose, LimelightPipeline pipeline) {
		this.name = name;
		this.logPath = logPathPrefix + "/" + name;

		this.robotRelativeCameraPose = robotRelativeCameraPose;
		setRobotRelativeCameraPose(robotRelativeCameraPose);

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
					LimelightConstants.DEFAULT_STANDARD_DEVIATIONS
				);
				megaTag2RobotPoseObservation.setObservationValues(
					megaTag2RobotPoseEstimate.timestampSeconds,
					megaTag2RobotPoseEstimate.pose,
					LimelightConstants.DEFAULT_STANDARD_DEVIATIONS
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

}
