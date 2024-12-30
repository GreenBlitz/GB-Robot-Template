package frc.robot.vision.multivisionsources;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.vision.data.AprilTagVisionData;
import frc.robot.vision.sources.GyroRequiringVisionSource;
import frc.robot.vision.GyroAngleValues;
import frc.robot.vision.sources.VisionSource;
import frc.robot.vision.sources.limelights.LimeLightSource;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class MultiAprilTagVisionSources extends MultiVisionSources<AprilTagVisionData> {

	private final Supplier<Rotation2d> gyroSupplier;
	private final Supplier<Rotation2d> headingOffsetSupplier;
	private boolean useGyroForPoseEstimating;

	@SafeVarargs
	public MultiAprilTagVisionSources(
		String logPath,
		Supplier<Rotation2d> gyroSupplier,
		Supplier<Rotation2d> headingOffsetSupplier,
		VisionSource<AprilTagVisionData>... visionSources
	) {
		this(logPath, gyroSupplier, headingOffsetSupplier, List.of(visionSources));
	}

	public MultiAprilTagVisionSources(
		String logPath,
		Supplier<Rotation2d> gyroSupplier,
		Supplier<Rotation2d> headingOffsetSupplier,
		List<VisionSource<AprilTagVisionData>> visionSources
	) {
		super(logPath, visionSources);
		this.gyroSupplier = gyroSupplier;
		this.headingOffsetSupplier = headingOffsetSupplier;
		this.useGyroForPoseEstimating = true;
		logBotPose();
		updateBotPoseInLimelight();
	}

	private void updateYawInLimelights(Rotation2d yaw) {
		for (VisionSource<AprilTagVisionData> visionSource : visionSources) {
			if (visionSource instanceof GyroRequiringVisionSource gyroRequiringVisionSource) {
				gyroRequiringVisionSource
					.updateGyroAngleValues(new GyroAngleValues(yaw, 0, Rotation2d.fromDegrees(0), 0, Rotation2d.fromDegrees(0), 0));
			}
		}
	}

	public ArrayList<Rotation2d> getRawEstimatedAngles() {
		ArrayList<Rotation2d> output = new ArrayList<>();
		for (VisionSource<AprilTagVisionData> visionSource : visionSources) {
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

	private void updateBotPoseInLimelight() {
		for (VisionSource<AprilTagVisionData> visionSource : visionSources) {
			if (visionSource instanceof LimeLightSource limeLightSource) {
				limeLightSource.useGyroForPoseEstimating(useGyroForPoseEstimating);
			}
		}
	}

	public void setUseGyroForPoseEstimating(boolean useGyroForPoseEstimating) {
		this.useGyroForPoseEstimating = useGyroForPoseEstimating;
		for (VisionSource<? extends AprilTagVisionData> visionSource : visionSources) {
			if (visionSource instanceof GyroRequiringVisionSource gyroRequiringVisionSource) {
				gyroRequiringVisionSource.useGyroForPoseEstimating(useGyroForPoseEstimating);
			}
		}
		logBotPose();
		updateBotPoseInLimelight();
	}

	public void switchBotPoses() {
		this.useGyroForPoseEstimating = !useGyroForPoseEstimating;
		logBotPose();
		updateBotPoseInLimelight();
	}

	private void logBotPose() {
		Logger.recordOutput(logPath + "botPose2", useGyroForPoseEstimating);
		Logger.recordOutput(logPath + "botPose1", !useGyroForPoseEstimating);
		Logger.recordOutput(logPath + "offsetedRobotHeading", getRobotHeading());
	}

	public void periodic() {
		super.log();
		updateYawInLimelights(getRobotHeading());
	}

	private Rotation2d getRobotHeading() {
		return gyroSupplier.get().plus(headingOffsetSupplier.get());
	}

}
