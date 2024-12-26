package frc.robot.vision.multivisionsources;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.vision.data.AprilTagVisionData;
import frc.robot.vision.sources.GyroRequiringVisionSource;
import frc.robot.vision.GyroAngleValues;
import frc.robot.vision.sources.VisionSource;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class MultiAprilTagVisionSource extends MultiVisionSources<AprilTagVisionData> {

	private final Supplier<Rotation2d> gyroSupplier;
	private final Supplier<Rotation2d> headingOffsetSupplier;
	private boolean useGyroForPoseEstimating;

	@SafeVarargs
	public MultiAprilTagVisionSource(
		String logPath,
		Supplier<Rotation2d> gyroSupplier,
		int angleInitializationSamplesCount, Supplier<Rotation2d> headingOffsetSupplier,
		VisionSource<AprilTagVisionData>... visionSources
	) {
		this(logPath, gyroSupplier, angleInitializationSamplesCount, List.of(visionSources), headingOffsetSupplier);
	}

	public MultiAprilTagVisionSource(
		String logPath,
		Supplier<Rotation2d> gyroSupplier,
		int angleInitializationSamplesCount,
		List<VisionSource<AprilTagVisionData>> visionSources, Supplier<Rotation2d> headingOffsetSupplier
	) {
		super(logPath, visionSources);
		this.gyroSupplier = gyroSupplier;
		this.headingOffsetSupplier = headingOffsetSupplier;
		this.useGyroForPoseEstimating = true;
		logBotPose();
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
		updateYawInLimelights(gyroSupplier.get().plus(headingOffsetSupplier.get()));
	}

}
