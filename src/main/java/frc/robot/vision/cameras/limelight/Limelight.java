package frc.robot.vision.cameras.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.vision.DetectedObjectObseration;
import frc.robot.vision.DetectedObjectType;
import frc.robot.vision.RobotPoseObservation;
import frc.robot.vision.interfaces.ObjectDetector;
import frc.robot.vision.interfaces.OrientationRequiringRobotPoseSupplier;
import frc.robot.vision.interfaces.IndependentRobotPoseSupplier;
import frc.utils.Conversions;
import frc.utils.LimelightHelpers;
import frc.utils.filter.Filter;
import frc.utils.math.StandardDeviations2D;
import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;
import java.util.function.Supplier;

public class Limelight implements ObjectDetector, IndependentRobotPoseSupplier, OrientationRequiringRobotPoseSupplier {

	private final String name;
	private final String logPath;
	private final Pose3d robotRelativeCameraPose;

	private DetectedObjectObseration detectedObjectObseration;

	private LimelightTarget2dValues target2dValues;

	private RobotPoseObservation mt1PoseObservation;
	private RobotPoseObservation mt2PoseObservation;

	private LimelightHelpers.PoseEstimate mt1RawData;
	private LimelightHelpers.PoseEstimate mt2RawData;

	private Filter detectedObjectFilter;
	private Filter mt1PoseFilter;
	private Filter mt2PoseFilter;

	private Supplier<StandardDeviations2D> calculateMT1StdDevs;
	private Supplier<StandardDeviations2D> calculateMT2StdDevs;

	private LimelightPipeline pipeline;

	public Limelight(String name, String logPathPrefix, Pose3d robotRelativeCameraPose, LimelightPipeline pipeline) {
		this.name = name;
		this.logPath = logPathPrefix + "/" + name;

		this.robotRelativeCameraPose = robotRelativeCameraPose;
		setRobotRelativeCameraPose(robotRelativeCameraPose);

		this.detectedObjectObseration = new DetectedObjectObseration();

		this.target2dValues = new LimelightTarget2dValues();

		this.mt1PoseObservation = new RobotPoseObservation();
		this.mt2PoseObservation = new RobotPoseObservation();

		this.mt1RawData = new LimelightHelpers.PoseEstimate();
		this.mt2RawData = new LimelightHelpers.PoseEstimate();

		this.detectedObjectFilter = Filter.nonFilteringFilter();
		this.mt1PoseFilter = Filter.nonFilteringFilter();
		this.mt2PoseFilter = Filter.nonFilteringFilter();

		this.calculateMT1StdDevs = () -> LimelightStdDevCalculations.DEFAULT_STD_DEVS;
		this.calculateMT2StdDevs = () -> LimelightStdDevCalculations.DEFAULT_STD_DEVS;

		setPipeline(pipeline);
	}

	public void log() {
		if (pipeline.isUsingMT()) {
			if (doesObservationExist(mt1PoseObservation)) {
				Logger.recordOutput(logPath + "/megaTag1PoseObservation", mt1PoseObservation);
			}
			if (doesObservationExist(mt2PoseObservation)) {
				Logger.recordOutput(logPath + "/megaTag2PoseObservation", mt2PoseObservation);
			}
		} else if (pipeline.isDetectingObjects()) {
			if (doesObservationExist(detectedObjectObseration)) {
				Logger.recordOutput(logPath + "/detectedObjectObservation", detectedObjectObseration);
			}
		}
	}

	public void updateObjectDetection() {
		target2dValues = LimelightTarget2dValues.fromArray(LimelightHelpers.getT2DArray(name));
		if (target2dValues.isValid()) {
			DetectedObjectType.getByName(LimelightHelpers.getDetectorClass(name))
				.ifPresent(
					objectType -> detectedObjectObseration = ObjectDetectionMath.getDetectedObjectObservation(
						robotRelativeCameraPose,
						objectType,
						target2dValues.targetX(),
						target2dValues.targetY(),
						getTarget2dTimestampSeconds(target2dValues)
					)
				);
		} else {
			detectedObjectObseration = new DetectedObjectObseration();
		}
	}

	public void updateMT1() {
		if (pipeline.isUsingMT()) {
			mt1RawData = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
			mt1PoseObservation = new RobotPoseObservation(getEstimateTimestampSeconds(mt1RawData), mt1RawData.pose, calculateMT1StdDevs.get());
		}
	}

	public void updateMT2() {
		if (pipeline.isUsingMT()) {
			mt2RawData = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
			mt2PoseObservation = new RobotPoseObservation(getEstimateTimestampSeconds(mt2RawData), mt2RawData.pose, calculateMT2StdDevs.get());
		}
	}

	public String getName() {
		return name;
	}

	public Pose3d getRobotRelativeCameraPose() {
		return robotRelativeCameraPose;
	}

	@Override
	public Optional<DetectedObjectObseration> getRobotRelativeObjectTranslation() {
		if (pipeline.isDetectingObjects() && doesObservationExist(detectedObjectObseration) && detectedObjectFilter.passesFilter()) {
			return Optional.of(detectedObjectObseration);
		}
		return Optional.empty();
	}

	@Override
	public Optional<RobotPoseObservation> getIndependentRobotPose() {
		if (pipeline.isUsingMT() && doesObservationExist(mt1PoseObservation) && mt1PoseFilter.passesFilter()) {
			return Optional.of(mt1PoseObservation);
		}
		return Optional.empty();
	}

	@Override
	public Optional<RobotPoseObservation> getOrientationRequiringRobotPose() {
		if (pipeline.isUsingMT() && doesObservationExist(mt2PoseObservation) && mt2PoseFilter.passesFilter()) {
			return Optional.of(mt2PoseObservation);
		}
		return Optional.empty();
	}

	public Filter getDetectedObjectFilter() {
		return detectedObjectFilter;
	}

	public Filter getMt1PoseFilter() {
		return mt1PoseFilter;
	}

	public Filter getMt2PoseFilter() {
		return mt2PoseFilter;
	}

	@Override
	public void setRobotOrientation(Rotation3d robotOrientation) {
		LimelightHelpers.SetRobotOrientation(
			name,
			Math.toDegrees(robotOrientation.getZ()),
			0,
			Math.toDegrees(robotOrientation.getY()),
			0,
			Math.toDegrees(robotOrientation.getX()),
			0
		);
	}

	public void setPipeline(LimelightPipeline pipeline) {
		this.pipeline = pipeline;
		LimelightHelpers.setPipelineIndex(name, pipeline.getPipelineIndex());
	}

	public void setDetectedObjectFilter(Filter detectedObjectFilter) {
		this.detectedObjectFilter = detectedObjectFilter;
	}

	public void setMT1PoseFilter(Filter mt1RobotPoseFilter) {
		this.mt1PoseFilter = mt1RobotPoseFilter;
	}

	public void setMT2PoseFilter(Filter mt2RobotPoseFilter) {
		this.mt2PoseFilter = mt2RobotPoseFilter;
	}

	public void setMT1StdDevsCalculation(Supplier<StandardDeviations2D> calculateMT1StdDevs) {
		this.calculateMT1StdDevs = calculateMT1StdDevs;
	}

	public void setMT2StdDevsCalculation(Supplier<StandardDeviations2D> calculateMT2StdDevs) {
		this.calculateMT2StdDevs = calculateMT2StdDevs;
	}

	protected LimelightTarget2dValues getTarget2dValues() {
		return target2dValues;
	}

	protected LimelightHelpers.PoseEstimate getMT1RawData() {
		return mt1RawData;
	}

	protected LimelightHelpers.PoseEstimate getMT2RawData() {
		return mt2RawData;
	}

	private void setRobotRelativeCameraPose(Pose3d robotRelativeCameraPose) {
		LimelightHelpers.setCameraPose_RobotSpace(
			name,
			robotRelativeCameraPose.getX(),
			robotRelativeCameraPose.getY(),
			robotRelativeCameraPose.getZ(),
			Math.toDegrees(robotRelativeCameraPose.getRotation().getX()),
			Math.toDegrees(robotRelativeCameraPose.getRotation().getY()),
			Math.toDegrees(robotRelativeCameraPose.getRotation().getZ())
		);
	}

	protected static double getEstimateTimestampSeconds(LimelightHelpers.PoseEstimate poseEstimate) {
		if (poseEstimate.timestampSeconds == 0) {
			return 0;
		}
		return TimeUtil.getCurrentTimeSeconds() - Conversions.milliSecondsToSeconds(poseEstimate.latency);
	}

	private static double getTarget2dTimestampSeconds(LimelightTarget2dValues target2dValues) {
		return TimeUtil.getCurrentTimeSeconds()
			- Conversions.milliSecondsToSeconds(target2dValues.targetLatencyMilliseconds() + target2dValues.captureLatencyMilliseconds());
	}

	private static boolean doesObservationExist(DetectedObjectObseration detectedObjectObseration) {
		return !detectedObjectObseration.robotRelativeObjectTranslation().equals(Translation2d.kZero);
	}

	private static boolean doesObservationExist(RobotPoseObservation robotPoseObservation) {
		return !robotPoseObservation.robotPose().equals(Pose2d.kZero);
	}

}
