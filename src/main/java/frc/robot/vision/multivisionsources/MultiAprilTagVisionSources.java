package frc.robot.vision.multivisionsources;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.constants.VisionConstants;
import frc.robot.vision.data.AprilTagVisionData;
import frc.robot.vision.sources.RobotHeadingRequiringVisionSource;
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
	private boolean useRobotHeadingForPoseEstimating;

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
		setUseRobotHeadingForPoseEstimating(!VisionConstants.REQUIRE_HEADING_TO_ESTIMATE_ANGLE);
	}

	private void updateYawInLimelights(Rotation2d yaw) {
		for (VisionSource<AprilTagVisionData> visionSource : visionSources) {
			if (visionSource instanceof RobotHeadingRequiringVisionSource gyroRequiringVisionSource) {
				gyroRequiringVisionSource
					.updateGyroAngleValues(new GyroAngleValues(yaw, 0, Rotation2d.fromDegrees(0), 0, Rotation2d.fromDegrees(0), 0));
			}
		}
	}

	public ArrayList<Rotation2d> getRawEstimatedAngles() {
		ArrayList<Rotation2d> output = new ArrayList<>();
		for (VisionSource<AprilTagVisionData> visionSource : visionSources) {
			if (visionSource instanceof RobotHeadingRequiringVisionSource gyroRequiringVisionSource) {
				gyroRequiringVisionSource.getRobotHeading().ifPresent(output::add);
			} else {
				visionSource.update();
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
				limeLightSource.useRobotHeadingForPoseEstimating(useRobotHeadingForPoseEstimating);
			}
		}
	}

	public void setUseRobotHeadingForPoseEstimating(boolean useRobotHeadingForPoseEstimating) {
		this.useRobotHeadingForPoseEstimating = useRobotHeadingForPoseEstimating;
		updateBotPoseInLimelight();
		logBotPose();
	}

	public void switchBotPoses() {
		setUseRobotHeadingForPoseEstimating(!useRobotHeadingForPoseEstimating);
	}

	@Override
	public ArrayList<AprilTagVisionData> getFilteredVisionData() {
		updateYawInLimelights(getRobotHeading());
		return super.getFilteredVisionData();
	}

	@Override
	public ArrayList<AprilTagVisionData> getUnfilteredVisionData() {
		updateYawInLimelights(getRobotHeading());
		return super.getUnfilteredVisionData();
	}

	private void logBotPose() {
		super.log();
		Logger.recordOutput(logPath + "botPose2", useRobotHeadingForPoseEstimating);
		Logger.recordOutput(logPath + "botPose1", !useRobotHeadingForPoseEstimating);
	}

	@Override
	protected void log() {
		super.log();
		Logger.recordOutput(logPath + "offsetedRobotHeading", getRobotHeading());
		Logger.recordOutput(logPath + "headingOffset", headingOffsetSupplier.get());
		Logger.recordOutput(logPath + "gyroInput", gyroSupplier.get());
	}

	private Rotation2d getRobotHeading() {
		return gyroSupplier.get().plus(headingOffsetSupplier.get());
	}

}
