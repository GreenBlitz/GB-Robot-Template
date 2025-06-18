package frc.robot.newvision.cameras.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.newvision.interfaces.RobotOrientationRequiringCamera;
import frc.robot.newvision.interfaces.RobotPoseSupplyingCamera;
import frc.utils.LimelightHelpers;
import frc.utils.TimedValue;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

public class Limelight implements RobotPoseSupplyingCamera, RobotOrientationRequiringCamera {

	private final String name;
	private final String logPath;
	private final Pose3d robotRelativeCameraPose;

	private LimelightPipeline pipeline;
	private LimelightHelpers.PoseEstimate megaTag1RobotPoseEstimate;
	private LimelightHelpers.PoseEstimate megaTag2RobotPoseEstimate;

	public Limelight(String name, String logPathPrefix, Pose3d robotRelativeCameraPose, LimelightPipeline pipeline) {
		this.name = name;
		this.logPath = logPathPrefix + "/" + name;

		this.robotRelativeCameraPose = robotRelativeCameraPose;
		setRobotRelativeCameraPose(robotRelativeCameraPose);

		setPipeline(pipeline);

		this.megaTag1RobotPoseEstimate = new LimelightHelpers.PoseEstimate();
		this.megaTag2RobotPoseEstimate = new LimelightHelpers.PoseEstimate();
	}

	public void update() {
		switch (pipeline) {
			case APRIL_TAG -> {
				megaTag1RobotPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
				megaTag2RobotPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
			}
			default -> {}
		}

		log();
	}

	private void log() {
		switch (pipeline) {
			case APRIL_TAG -> {
				Logger.recordOutput(logPath + "/megaTag1Pose", megaTag1RobotPoseEstimate.pose);
				Logger.recordOutput(logPath + "/megaTag2Pose", megaTag2RobotPoseEstimate.pose);
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

	public LimelightHelpers.PoseEstimate getMegaTag1RobotPoseEstimate() {
		return megaTag1RobotPoseEstimate;
	}

	public LimelightHelpers.PoseEstimate getMegaTag2RobotPoseEstimate() {
		return megaTag2RobotPoseEstimate;
	}

	@Override
	public Optional<TimedValue<Pose2d>> getRobotPose() {
		if (pipeline.equals(LimelightPipeline.APRIL_TAG)) {
			return Optional.of(new TimedValue<>(megaTag1RobotPoseEstimate.pose, megaTag1RobotPoseEstimate.timestampSeconds));
		}
		return Optional.empty();
	}

	@Override
	public Optional<TimedValue<Pose2d>> getOrientationRequiringRobotPose() {
		if (pipeline.equals(LimelightPipeline.APRIL_TAG)) {
			return Optional.of(new TimedValue<>(megaTag2RobotPoseEstimate.pose, megaTag2RobotPoseEstimate.timestampSeconds));
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
