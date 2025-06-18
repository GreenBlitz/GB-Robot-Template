package frc.robot.newvision.cameras.limelight;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.newvision.interfaces.ObjectDetector;
import frc.robot.newvision.interfaces.RobotYawRequiringCamera;
import frc.robot.newvision.interfaces.RobotPoseSupplyingCamera;
import frc.utils.LimelightHelpers;
import frc.utils.TimedValue;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

public class Limelight implements RobotPoseSupplyingCamera, RobotYawRequiringCamera, ObjectDetector {

	private final String name;

	private final String logPath;

	private final Pose3d robotRelativeCameraPose;

	private final LimelightTarget2dValues target2dValues;

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
		this.target2dValues = new LimelightTarget2dValues();
	}

	public void update() {
		switch (pipeline) {
			case APRIL_TAG -> {
				megaTag1RobotPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
				megaTag2RobotPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
			}
			case OBJECT_DETECTION -> {
				target2dValues.setValues(LimelightHelpers.getT2DArray(name), logPath);
			}
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
	public Optional<Translation2d> getRobotRelativeObjectTranslation() {
		return Optional.empty(); // todo implement
	}

	@Override
	public TimedValue<Pose3d> getRobotPose() {
		return new TimedValue<>(
			LimelightHelpers.toPose3D(LimelightHelpers.pose2dToArray(megaTag1RobotPoseEstimate.pose)),
			megaTag1RobotPoseEstimate.timestampSeconds
		);
	}

	@Override
	public TimedValue<Pose3d> getYawRequiringRobotPose() {
		return new TimedValue<>(
			LimelightHelpers.toPose3D(LimelightHelpers.pose2dToArray(megaTag2RobotPoseEstimate.pose)),
			megaTag2RobotPoseEstimate.timestampSeconds
		);
	}

	@Override
	public void setRobotYaw(Rotation2d robotYaw) {
		LimelightHelpers.SetRobotOrientation(name, robotYaw.getDegrees(), 0, 0, 0, 0, 0);
	}

	public void setPipeline(LimelightPipeline pipeline) {
		this.pipeline = pipeline;
		LimelightHelpers.setPipelineIndex(name, pipeline.ordinal());
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
