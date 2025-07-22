package frc.robot.vision.multivisionsources;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.vision.VisionConstants;
import frc.utils.TimedValue;
import frc.robot.vision.data.AprilTagVisionData;
import frc.robot.vision.sources.IndpendentHeadingVisionSource;
<<<<<<< HEAD
import frc.robot.vision.RobotAngleValues;
=======
import frc.robot.vision.OrientationState3D;
>>>>>>> template/master
import frc.robot.vision.sources.RobotHeadingRequiringVisionSource;
import frc.robot.vision.sources.VisionSource;
import frc.robot.vision.sources.limelights.DynamicSwitchingLimelight;
import frc.robot.vision.sources.limelights.LimeLightSource;
import frc.utils.alerts.Alert;
import frc.utils.pose.PoseUtil;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;
import java.util.function.Supplier;

/**
 * Extended MultiVisionSources that supplies methods that takes care of using, updating and extracting data from special interfaces related
 * specifically to sources that detect april tags, e.g. <code>IndependentHeadingVisionSource</code>.
 *
 * <p>This class assumes that the robot has zero pitch and roll.
 */
public class MultiAprilTagVisionSources extends MultiVisionSources<AprilTagVisionData> {

	private final Supplier<Rotation2d> robotHeadingSupplier;
	private boolean useRobotHeadingForPoseEstimating;

	public MultiAprilTagVisionSources(
		String logPath,
		Supplier<Rotation2d> robotHeadingSupplier,
		boolean useRobotHeadingForPoseEstimating,
		List<VisionSource<AprilTagVisionData>> visionSources
	) {
		super(logPath, visionSources);
		this.robotHeadingSupplier = robotHeadingSupplier;
		setUseRobotHeadingForPoseEstimating(useRobotHeadingForPoseEstimating);
	}

	@SafeVarargs
	public MultiAprilTagVisionSources(
		String logPath,
		Supplier<Rotation2d> robotHeadingSupplier,
		boolean useRobotHeadingForPoseEstimating,
		VisionSource<AprilTagVisionData>... visionSources
	) {
		this(logPath, robotHeadingSupplier, useRobotHeadingForPoseEstimating, List.of(visionSources));
	}

<<<<<<< HEAD
	private void updateAngleInHeadingRequiringSources(RobotAngleValues robotAngleValues) {
		for (VisionSource<AprilTagVisionData> visionSource : visionSources) {
			if (visionSource instanceof RobotHeadingRequiringVisionSource robotHeadingRequiringVisionSource) {
				robotHeadingRequiringVisionSource.updateRobotAngleValues(robotAngleValues);
=======
	private void updateAngleInHeadingRequiringSources(OrientationState3D robotOrientationState) {
		for (VisionSource<AprilTagVisionData> visionSource : visionSources) {
			if (visionSource instanceof RobotHeadingRequiringVisionSource robotHeadingRequiringVisionSource) {
				robotHeadingRequiringVisionSource.updateRobotAngleValues(robotOrientationState);
>>>>>>> template/master
			}
		}
	}

<<<<<<< HEAD
	private void updateAngleInHeadingRequiringSources(Rotation3d angle, double yawRate, double pitchRate, double rollRate) {
		updateAngleInHeadingRequiringSources(new RobotAngleValues(angle, yawRate, pitchRate, rollRate));
	}

	private void updateAngleInHeadingRequiringSources(Rotation3d angle) {
		updateAngleInHeadingRequiringSources(new RobotAngleValues(angle));
=======
	private void updateAngleInHeadingRequiringSources(Rotation3d angle, Rotation3d angularVelocity) {
		updateAngleInHeadingRequiringSources(new OrientationState3D(angle, angularVelocity));
	}

	private void updateAngleInHeadingRequiringSources(Rotation3d angle) {
		updateAngleInHeadingRequiringSources(new OrientationState3D(angle));
>>>>>>> template/master
	}

	private void updateAngleInHeadingRequiringSources(Rotation2d yaw) {
		updateAngleInHeadingRequiringSources(new Rotation3d(0, 0, yaw.getRadians()));
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

	private void updateMegaTagMethodsUsedInDynamicLimelights() {
		for (VisionSource<AprilTagVisionData> visionSource : visionSources) {
			if (visionSource instanceof DynamicSwitchingLimelight dynamicSwitchingLimelight) {
				dynamicSwitchingLimelight.setUseRobotHeadingForPoseEstimating(useRobotHeadingForPoseEstimating);
			} else if (visionSource instanceof LimeLightSource) {
				new Alert(Alert.AlertType.WARNING, "unableToSwitchMegaTagMethodInNonDynamicLimelight").report();
			}
		}
	}

	public void setUseRobotHeadingForPoseEstimating(boolean useRobotHeadingForPoseEstimating) {
		this.useRobotHeadingForPoseEstimating = useRobotHeadingForPoseEstimating;
		updateMegaTagMethodsUsedInDynamicLimelights();
		logMegaTagMethod();
	}

	public void switchMegaTagCalculationMethod() {
		setUseRobotHeadingForPoseEstimating(!useRobotHeadingForPoseEstimating);
	}

	@Override
	public ArrayList<AprilTagVisionData> getFilteredVisionData() {
		updateAngleInHeadingRequiringSources(robotHeadingSupplier.get());
		return super.getFilteredVisionData();
	}

	@Override
	public ArrayList<AprilTagVisionData> getUnfilteredVisionData() {
		updateAngleInHeadingRequiringSources(robotHeadingSupplier.get());
		return super.getUnfilteredVisionData();
	}

	private void logMegaTagMethod() {
//		Logger.recordOutput(logPath + "isMegaTag1InUse", !useRobotHeadingForPoseEstimating);
//		Logger.recordOutput(logPath + "isMegaTag2InUse", useRobotHeadingForPoseEstimating);
	}

	private void logAprilTagPoseData() {
<<<<<<< HEAD
		List<Integer> seenApriltagIDs = new ArrayList<>();
=======
		List<Integer> seenAprilTagsIDs = new ArrayList<>();
>>>>>>> template/master
		for (AprilTagVisionData visionData : getUnfilteredVisionData()) {
			int aprilTagID = visionData.getTrackedAprilTagId();
			Optional<Pose3d> aprilTag = VisionConstants.APRIL_TAG_FIELD_LAYOUT.getTagPose(aprilTagID);
			aprilTag.ifPresent((pose) -> {
				Logger.recordOutput(logPath + "targets/" + aprilTagID, pose);
<<<<<<< HEAD
				seenApriltagIDs.add(aprilTagID);
			});
		}
		for (int aprilTagID = 1; aprilTagID <= VisionConstants.APRIL_TAG_FIELD_LAYOUT.getTags().size(); aprilTagID++) {
			if (!seenApriltagIDs.contains(aprilTagID)) {
				Logger.recordOutput(logPath + "targets/" + aprilTagID, PoseUtil.EMPTY_POSE2D);
=======
				seenAprilTagsIDs.add(aprilTagID);
			});
		}
		for (int aprilTagID = 1; aprilTagID <= VisionConstants.APRIL_TAG_FIELD_LAYOUT.getTags().size(); aprilTagID++) {
			if (!seenAprilTagsIDs.contains(aprilTagID)) {
				Logger.recordOutput(logPath + "targets/" + aprilTagID, Pose2d.kZero);
>>>>>>> template/master
			}
		}
	}

	/* Costly on runtime: don't use until gets fixed */
	@Override
	public void log() {
		super.log();
		logAprilTagPoseData();
		Logger.recordOutput(logPath + "inputtedRobotHeading", robotHeadingSupplier.get());
	}

}
