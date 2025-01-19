package frc.robot.vision.multivisionsources;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.constants.VisionConstants;
import frc.robot.hardware.signal.TimedValue;
import frc.robot.vision.data.AprilTagVisionData;
import frc.robot.vision.sources.IndpendentHeadingVisionSource;
import frc.robot.vision.GyroAngleValues;
import frc.robot.vision.sources.RobotHeadingRequiringVisionSource;
import frc.robot.vision.sources.VisionSource;
import frc.robot.vision.sources.limelights.DynamicSwitchingLimelight;
import frc.utils.alerts.Alert;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;
import java.util.function.Supplier;

/**
 * Extended MultiVisionSources that supplies methods that takes care of using, updating and extracting data from special interfaces related
 * specifically to sources that detect april tags, e.g. `IndpendentHeadingVisionSource`.
 *
 * This class assumes that the robot has zero pitch and roll.
 */
public class MultiAprilTagVisionSources extends MultiVisionSources<AprilTagVisionData> {

	private final Supplier<Rotation2d> gyroSupplier;
	private final Supplier<Rotation2d> headingOffsetSupplier;
	private boolean useRobotHeadingForPoseEstimating;

	public MultiAprilTagVisionSources(
		String logPath,
		Supplier<Rotation2d> gyroSupplier,
		Supplier<Rotation2d> headingOffsetSupplier,
		List<VisionSource<AprilTagVisionData>> visionSources
	) {
		super(logPath, visionSources);
		this.gyroSupplier = gyroSupplier;
		this.headingOffsetSupplier = headingOffsetSupplier;
		setUseRobotHeadingForPoseEstimating(VisionConstants.REQUIRE_HEADING_TO_ESTIMATE_ANGLE_DEFAULT_VALUE);
	}

	@SafeVarargs
	public MultiAprilTagVisionSources(
		String logPath,
		Supplier<Rotation2d> gyroSupplier,
		Supplier<Rotation2d> headingOffsetSupplier,
		VisionSource<AprilTagVisionData>... visionSources
	) {
		this(logPath, gyroSupplier, headingOffsetSupplier, List.of(visionSources));
	}

	private void updateAngleInHeadingRequiringSources(GyroAngleValues gyroAngleValues) {
		for (VisionSource<AprilTagVisionData> visionSource : visionSources) {
			if (visionSource instanceof RobotHeadingRequiringVisionSource robotHeadingRequiringVisionSource) {
				robotHeadingRequiringVisionSource.updateGyroAngleValues(gyroAngleValues);
			}
		}
	}

	private void updateAngleInHeadingRequiringSources(Rotation3d angle, double yawRate, double pitchRate, double rollRate) {
		updateAngleInHeadingRequiringSources(new GyroAngleValues(angle, yawRate, pitchRate, rollRate));
	}

	private void updateAngleInHeadingRequiringSources(Rotation3d angle) {
		updateAngleInHeadingRequiringSources(new GyroAngleValues(angle));
	}

	private void updateAngleInHeadingRequiringSources(Rotation2d yaw) {
		updateAngleInHeadingRequiringSources(new Rotation3d(yaw.getRadians(), 0, 0));
	}

	protected ArrayList<TimedValue<Rotation2d>> extractHeadingDataFromMappedSources(
		List<VisionSource<AprilTagVisionData>> sources,
		Function<IndpendentHeadingVisionSource, Optional<TimedValue<Rotation2d>>> mapping
	) {
		ArrayList<TimedValue<Rotation2d>> output = new ArrayList<>();
		for (VisionSource<AprilTagVisionData> visionSource : sources) {
			if (visionSource instanceof IndpendentHeadingVisionSource indpendentHeadingVisionSource) {
				mapping.apply(indpendentHeadingVisionSource).ifPresent(output::add);
			}
		}
		return output;
	}

	public ArrayList<TimedValue<Rotation2d>> getRawRobotHeadings() {
		return extractHeadingDataFromMappedSources(visionSources, IndpendentHeadingVisionSource::getRawHeadingData);
	}

	public ArrayList<TimedValue<Rotation2d>> getFilteredRobotHeading() {
		return extractHeadingDataFromMappedSources(visionSources, IndpendentHeadingVisionSource::getFilteredHeadingData);
	}

	private void updateMegaTagInDynamicLimelights() {
		for (VisionSource<AprilTagVisionData> visionSource : visionSources) {
			if (visionSource instanceof DynamicSwitchingLimelight dynamicSwitchingLimelight) {
				dynamicSwitchingLimelight.setUseRobotHeadingForPoseEstimating(useRobotHeadingForPoseEstimating);
			} else if (!useRobotHeadingForPoseEstimating) {
				new Alert(Alert.AlertType.WARNING, "unableToSwitchMegaTagsInNonDynamicLimelight").report();
			}
		}
	}

	public void setUseRobotHeadingForPoseEstimating(boolean useRobotHeadingForPoseEstimating) {
		this.useRobotHeadingForPoseEstimating = useRobotHeadingForPoseEstimating;
		updateMegaTagInDynamicLimelights();
		logMegaTagMethod();
	}

	public void switchMegaTagCalculationMethod() {
		setUseRobotHeadingForPoseEstimating(!useRobotHeadingForPoseEstimating);
	}

	@Override
	public ArrayList<AprilTagVisionData> getFilteredVisionData() {
		updateAngleInHeadingRequiringSources(getRobotHeading());
		return super.getFilteredVisionData();
	}

	@Override
	public ArrayList<AprilTagVisionData> getUnfilteredVisionData() {
		updateAngleInHeadingRequiringSources(getRobotHeading());
		return super.getUnfilteredVisionData();
	}

	private void logMegaTagMethod() {
		Logger.recordOutput(logPath + "isMegaTag1InUse", !useRobotHeadingForPoseEstimating);
		Logger.recordOutput(logPath + "isMegaTag2InUse", useRobotHeadingForPoseEstimating);
	}

	private void logAprilTagPoseData() {
		for (AprilTagVisionData visionData : getUnfilteredVisionData()) {
			int aprilTagID = visionData.getTrackedAprilTagId();
			Optional<Pose3d> aprilTag = VisionConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose(aprilTagID);
			aprilTag.ifPresent((pose) -> Logger.recordOutput(logPath + "targets/" + aprilTagID, pose));
		}
	}

	@Override
	public void log() {
		super.log();
		logAprilTagPoseData();
		Logger.recordOutput(logPath + "offsettedRobotHeading", getRobotHeading());
		Logger.recordOutput(logPath + "headingOffset", headingOffsetSupplier.get());
		Logger.recordOutput(logPath + "gyroInput", gyroSupplier.get());
	}

	private Rotation2d getRobotHeading() {
		return gyroSupplier.get().plus(headingOffsetSupplier.get());
	}

}
