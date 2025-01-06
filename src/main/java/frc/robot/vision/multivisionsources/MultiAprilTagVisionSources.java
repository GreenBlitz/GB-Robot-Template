package frc.robot.vision.multivisionsources;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
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
import java.util.Optional;
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
		setUseRobotHeadingForPoseEstimating(VisionConstants.REQUIRE_HEADING_TO_ESTIMATE_ANGLE);
	}

	private void updateYawInLimelights(Rotation2d yaw) {
		for (VisionSource<AprilTagVisionData> visionSource : visionSources) {
			if (visionSource instanceof RobotHeadingRequiringVisionSource gyroRequiringVisionSource) {
//				Logger.recordOutput(logPath + "Robot Heading", gyroSupplier.get());
				gyroRequiringVisionSource
					.updateGyroAngleValues(new GyroAngleValues(yaw, 0, Rotation2d.fromDegrees(0), 0, Rotation2d.fromDegrees(0), 0));
			}
		}
	}

	public ArrayList<Pair<Rotation2d, Double>> getRawEstimatedAngles() {
		ArrayList<Pair<Rotation2d, Double>> output = new ArrayList<>();
		for (VisionSource<AprilTagVisionData> visionSource : visionSources) {
			if (visionSource instanceof RobotHeadingRequiringVisionSource gyroRequiringVisionSource) {
				gyroRequiringVisionSource.getRobotHeading().ifPresent(output::add);
			} else {
				visionSource.update();
				visionSource.getVisionData()
					.ifPresent(
						(AprilTagVisionData visionData) -> output.add(new Pair<>(
							Rotation2d.fromRadians(visionData.getEstimatedPose().getRotation().getZ()),
							visionData.getTimestamp()
						))
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
		Logger.recordOutput(logPath + "usingBotPose2", useRobotHeadingForPoseEstimating);
		Logger.recordOutput(logPath + "usingBotPose1", !useRobotHeadingForPoseEstimating);
	}

	private void logObservationData() {
		for (AprilTagVisionData visionData : getUnfilteredVisionData()) {
			int aprilTagID = visionData.getTrackedAprilTagId();
			Optional<Pose3d> aprilTag = VisionConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose(aprilTagID);
			aprilTag.ifPresent((pose) -> Logger.recordOutput(logPath + "targets/" + aprilTagID, pose));
		}
	}

	@Override
	public void log() {
		super.log();
		logObservationData();
		Logger.recordOutput(logPath + "offsetedRobotHeading", getRobotHeading());
		Logger.recordOutput(logPath + "headingOffset", headingOffsetSupplier.get());
		Logger.recordOutput(logPath + "gyroInput", gyroSupplier.get());
	}

	private Rotation2d getRobotHeading() {
		return gyroSupplier.get().plus(headingOffsetSupplier.get());
	}

}
