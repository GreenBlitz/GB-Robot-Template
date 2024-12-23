package frc.robot.vision.multivisionsources;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.vision.data.AprilTagVisionData;
import frc.robot.vision.sources.GyroRequiringVisionSource;
import frc.robot.vision.GyroAngleValues;
import frc.robot.vision.sources.VisionSource;
import frc.utils.GBMeth;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.Supplier;

public class MultiAprilTagVisionSource extends MultiVisionSources<AprilTagVisionData> {

	private final Supplier<Rotation2d> gyroSupplier;
	private final ArrayBlockingQueue<Rotation2d> rotationAccumulator;
	private final int angleInitializationSamplesCount;
	private boolean useGyroForPoseEstimating;
	private Rotation2d headingOffset;
	private boolean hasHeadingOffsetBeenInitialized;

	@SafeVarargs
	public MultiAprilTagVisionSource(
		String logPath,
		Supplier<Rotation2d> gyroSupplier,
		int angleInitializationSamplesCount,
		VisionSource<AprilTagVisionData>... visionSources
	) {
		this(logPath, gyroSupplier, angleInitializationSamplesCount, List.of(visionSources));
	}

	public MultiAprilTagVisionSource(
		String logPath,
		Supplier<Rotation2d> gyroSupplier,
		int angleInitializationSamplesCount,
		List<VisionSource<AprilTagVisionData>> visionSources
	) {
		super(logPath, visionSources);
		this.gyroSupplier = gyroSupplier;
		this.rotationAccumulator = new ArrayBlockingQueue<>(angleInitializationSamplesCount);
		this.useGyroForPoseEstimating = true;
		this.angleInitializationSamplesCount = angleInitializationSamplesCount;
		this.headingOffset = Rotation2d.fromDegrees(0);
		this.hasHeadingOffsetBeenInitialized = false;
		logBotPose();
	}

	private void attemptToCalculateHeadingOffset() {
		Optional<Rotation2d> yaw = GBMeth.calculateAngleAverage(rotationAccumulator.stream().toList());
		if (yaw.isPresent()) {
			headingOffset = yaw.get().minus(gyroSupplier.get());
			hasHeadingOffsetBeenInitialized = false;
		}
	}

	private void updateYawInLimelights(Rotation2d yaw) {
		for (VisionSource<AprilTagVisionData> visionSource : getVisionSources()) {
			if (visionSource instanceof GyroRequiringVisionSource gyroRequiringVisionSource) {
				gyroRequiringVisionSource
					.updateGyroAngleValues(new GyroAngleValues(yaw, 0, Rotation2d.fromDegrees(0), 0, Rotation2d.fromDegrees(0), 0));
			}
		}
	}

	public ArrayList<Rotation2d> getRawEstimatedAngles() {
		ArrayList<Rotation2d> output = new ArrayList<>();
		for (VisionSource<AprilTagVisionData> visionSource : getVisionSources()) {
			if (visionSource instanceof GyroRequiringVisionSource gyroRequiringVisionSource) {
				gyroRequiringVisionSource.getRobotHeading().ifPresent(output::add);
			} else {
				visionSource.getVisionData()
					.ifPresent(
						(AprilTagVisionData visionData) -> output.add(Rotation2d.fromRadians(visionData.getEstimatedPose().getRotation().getZ()))
					);
			}
		}
		return output;
	}

	public void setUseGyroForPoseEstimating(boolean useGyroForPoseEstimating) {
		this.useGyroForPoseEstimating = useGyroForPoseEstimating;
		for (VisionSource<? extends AprilTagVisionData> visionSource : getVisionSources()) {
			if (visionSource instanceof GyroRequiringVisionSource gyroRequiringVisionSource) {
				gyroRequiringVisionSource.useGyroForPoseEstimating(useGyroForPoseEstimating);
			}
		}
		logBotPose();
	}

	public void switchBotPoses() {
		this.useGyroForPoseEstimating = !useGyroForPoseEstimating;
		logBotPose();
	}

	private void logBotPose() {
		Logger.recordOutput(logPath + "botPose2", useGyroForPoseEstimating);
		Logger.recordOutput(logPath + "botPose1", !useGyroForPoseEstimating);
	}

	public void periodic() {
		if (rotationAccumulator.size() >= angleInitializationSamplesCount && !hasHeadingOffsetBeenInitialized) {
			for (Rotation2d angle : getRawEstimatedAngles()) {
				rotationAccumulator.offer(angle);
			}
			attemptToCalculateHeadingOffset();
			updateYawInLimelights(gyroSupplier.get().plus(headingOffset));
		}
	}

}
