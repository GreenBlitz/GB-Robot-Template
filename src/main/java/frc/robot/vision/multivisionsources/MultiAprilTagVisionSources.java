package frc.robot.vision.multivisionsources;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.constants.VisionConstants;
import frc.robot.vision.data.AprilTagVisionData;
import frc.robot.vision.data.HeadingData;
import frc.robot.vision.sources.IndpendentHeadingVisionSource;
import frc.robot.vision.GyroAngleValues;
import frc.robot.vision.sources.RobotHeadingRequiringVisionSource;
import frc.robot.vision.sources.VisionSource;
import frc.robot.vision.sources.limelights.LimelightSources;
import frc.utils.alerts.Alert;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;
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
			if (visionSource instanceof RobotHeadingRequiringVisionSource robotHeadingRequiringVisionSource) {
				robotHeadingRequiringVisionSource
					.updateGyroAngleValues(new GyroAngleValues(yaw, 0, Rotation2d.fromDegrees(0), 0, Rotation2d.fromDegrees(0), 0));
			}
		}
	}

	protected ArrayList<HeadingData> extractHeadingDataFromMappedSources(
		List<VisionSource<AprilTagVisionData>> sources,
		Function<IndpendentHeadingVisionSource, Optional<AprilTagVisionData>> mapping
	) {
		ArrayList<HeadingData> output = new ArrayList<>();
		for (VisionSource<AprilTagVisionData> visionSource : sources) {
			if (visionSource instanceof IndpendentHeadingVisionSource indpendentHeadingVisionSource) {
				mapping.apply(indpendentHeadingVisionSource)
					.ifPresent(
						(visionData) -> output
							.add(new HeadingData(visionData.getEstimatedPose().getRotation().toRotation2d(), visionData.getTimestamp()))
					);
			}
		}
		return output;
	}

	public ArrayList<HeadingData> getRawRobotHeadings() {
		return extractHeadingDataFromMappedSources(visionSources, IndpendentHeadingVisionSource::getVisionData);
	}

	public ArrayList<HeadingData> getFilteredRobotHeading() {
		return extractHeadingDataFromMappedSources(visionSources, IndpendentHeadingVisionSource::getFilteredVisionData);
	}

	private void updateBotPoseInDynamicLimelights() {
		for (VisionSource<AprilTagVisionData> visionSource : visionSources) {
			if (visionSource instanceof LimelightSources.DynamicLimelight dynamicLimelight) {
				dynamicLimelight.useRobotHeadingForPoseEstimating(useRobotHeadingForPoseEstimating);
			} else if (
				visionSource instanceof LimelightSources.GyroSupportingLimelight || visionSource instanceof LimelightSources.NoisyLimelight
			) {
				new Alert(Alert.AlertType.WARNING, "unableToSwitchBotPoseOnNonDynamicLimelight");
			}
		}
	}

	public void setUseRobotHeadingForPoseEstimating(boolean useRobotHeadingForPoseEstimating) {
		this.useRobotHeadingForPoseEstimating = useRobotHeadingForPoseEstimating;
		updateBotPoseInDynamicLimelights();
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
		Logger.recordOutput(logPath + "isBotPose2InUse", useRobotHeadingForPoseEstimating);
		Logger.recordOutput(logPath + "isBotPose1InUse", !useRobotHeadingForPoseEstimating);
	}

	private void logTargetData() {
		for (AprilTagVisionData visionData : getUnfilteredVisionData()) {
			int aprilTagID = visionData.getTrackedAprilTagId();
			Optional<Pose3d> aprilTag = VisionConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose(aprilTagID);
			aprilTag.ifPresent((pose) -> Logger.recordOutput(logPath + "targets/" + aprilTagID, pose));
		}
	}

	@Override
	public void log() {
		super.log();
		logTargetData();
		Logger.recordOutput(logPath + "offsettedRobotHeading", getRobotHeading());
		Logger.recordOutput(logPath + "headingOffset", headingOffsetSupplier.get());
		Logger.recordOutput(logPath + "gyroInput", gyroSupplier.get());
	}

	private Rotation2d getRobotHeading() {
		return gyroSupplier.get().plus(headingOffsetSupplier.get());
	}

}
