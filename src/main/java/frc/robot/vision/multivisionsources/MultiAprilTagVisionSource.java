package frc.robot.vision.multivisionsources;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.vision.rawdata.AprilTagVisionData;
import frc.robot.vision.sources.LimeLightSource;
import frc.robot.vision.sources.LimelightGyroAngleValues;
import frc.robot.vision.sources.VisionSource;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;

public class MultiAprilTagVisionSource extends MultiVisionSources<AprilTagVisionData> {

	private final List<VisionSource<AprilTagVisionData>> visionSources;

	@SafeVarargs
	public MultiAprilTagVisionSource(String logPath, VisionSource<AprilTagVisionData>... visionSources) {
		super(logPath, visionSources);
		this.visionSources = List.of(visionSources);
		logBotPose(false);
	}

	public MultiAprilTagVisionSource(String logPath, List<VisionSource<AprilTagVisionData>> visionSources) {
		super(logPath, visionSources);
		this.visionSources = visionSources;
		logBotPose(false);
	}

	public void updateYawInLimelights(Rotation2d yaw) {
		for (VisionSource<AprilTagVisionData> visionSource : getVisionSources()) {
			if (visionSource instanceof LimeLightSource limelightSource) {
				limelightSource
					.updateGyroAngles(new LimelightGyroAngleValues(yaw, 0, Rotation2d.fromDegrees(0), 0, Rotation2d.fromDegrees(0), 0));
			}
		}
	}

	public ArrayList<Rotation2d> getRawEstimatedAngles() {
		ArrayList<Rotation2d> output = new ArrayList<>();
		for (VisionSource<AprilTagVisionData> visionSource : getVisionSources()) {
			if (visionSource instanceof LimeLightSource limeLightSource) {
				limeLightSource.getRobotHeading().ifPresent(output::add);
			} else {
				visionSource.getVisionData()
					.ifPresent(
						(AprilTagVisionData visionData) -> output.add(Rotation2d.fromRadians(visionData.getEstimatedPose().getRotation().getZ()))
					);
			}
		}
		return output;
	}

	public void switchToBotPose(boolean useBotPose1) {
		for (VisionSource<? extends AprilTagVisionData> visionSource : getVisionSources()) {
			if (visionSource instanceof LimeLightSource limelightSource) {
				limelightSource.changedUsedBotPoseVersion(useBotPose1);
			}
		}
		logBotPose(useBotPose1);
	}

	private void logBotPose(boolean useBotPose1) {
		Logger.recordOutput(getLogPath() + "botPose1", useBotPose1);
		Logger.recordOutput(getLogPath() + "botPose2", !useBotPose1);
	}

}
