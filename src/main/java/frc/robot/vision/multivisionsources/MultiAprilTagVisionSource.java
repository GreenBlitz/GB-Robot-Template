package frc.robot.vision.multivisionsources;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.vision.rawdata.AprilTagVisionData;
import frc.robot.vision.sources.LimeLightSource;
import frc.robot.vision.sources.LimelightGyroAngleValues;
import frc.robot.vision.sources.VisionSource;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.Supplier;

public class MultiAprilTagVisionSource extends MultiVisionSources<AprilTagVisionData> {

	private final Supplier<Rotation2d> gyroSupplier;
	private final List<VisionSource<AprilTagVisionData>> visionSources;
	private final ArrayBlockingQueue<Rotation2d> rotationsCountHelper;
	private boolean useBotPose1;

	@SafeVarargs
	public MultiAprilTagVisionSource(String logPath, Supplier<Rotation2d> gyroSupplier, int angleInitializationSamplesCount, VisionSource<AprilTagVisionData>... visionSources) {
		super(logPath, visionSources);
		this.gyroSupplier = gyroSupplier;
		this.visionSources = List.of(visionSources);
		this.rotationsCountHelper = new ArrayBlockingQueue<>(angleInitializationSamplesCount);
		this.useBotPose1 = false;
		logBotPose();
	}

	public MultiAprilTagVisionSource(String logPath, Supplier<Rotation2d> gyroSupplier, int angleInitializationSamplesCount, List<VisionSource<AprilTagVisionData>> visionSources) {
		super(logPath, visionSources);
		this.gyroSupplier = gyroSupplier;
		this.visionSources = visionSources;
		this.rotationsCountHelper = new ArrayBlockingQueue<>(angleInitializationSamplesCount);
		this.useBotPose1 = false;
		logBotPose();
	}

	private void updateYawInLimelights(Rotation2d yaw) {
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

	public void setUsedBotPose(boolean useBotPose1) {
		this.useBotPose1 = useBotPose1;
		for (VisionSource<? extends AprilTagVisionData> visionSource : getVisionSources()) {
			if (visionSource instanceof LimeLightSource limelightSource) {
				limelightSource.changedUsedBotPoseVersion(useBotPose1);
			}
		}
		logBotPose();
	}

	public void switchBotPoses() {
		this.useBotPose1 = !useBotPose1;
		logBotPose();
	}

	private void logBotPose() {
		Logger.recordOutput(getLogPath() + "botPose1", useBotPose1);
		Logger.recordOutput(getLogPath() + "botPose2", !useBotPose1);
	}

}
