package frc.robot.vision.cameras.limelight;

import edu.wpi.first.math.geometry.*;
import frc.robot.vision.DetectedObjectObservation;
import frc.robot.vision.RobotPoseObservation;
import frc.robot.vision.cameras.limelight.inputs.LimelightInputsSet;
import frc.robot.vision.interfaces.ObjectDetector;
import frc.robot.vision.interfaces.OrientationRequiringRobotPoseSupplier;
import frc.robot.vision.interfaces.IndependentRobotPoseSupplier;
import frc.utils.Conversions;
import frc.utils.LimelightHelpers;
import frc.utils.filter.Filter;
import frc.utils.math.StandardDeviations2D;
import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Function;
import java.util.function.Supplier;

public class Limelight implements ObjectDetector, IndependentRobotPoseSupplier, OrientationRequiringRobotPoseSupplier {

	private final String name;
	private final String logPath;
	private final Pose3d robotRelativeCameraPose;
	private final ArrayList<DetectedObjectObservation> detectedObjectObservations;

	private final LimelightInputsSet inputs;

	private RobotPoseObservation mt1PoseObservation;
	private RobotPoseObservation mt2PoseObservation;

	private Function<LimelightHelpers.RawDetection, Boolean> detectedObjectFilter;
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

		this.detectedObjectObservations = new ArrayList<>();

		this.mt1PoseObservation = new RobotPoseObservation();
		this.mt2PoseObservation = new RobotPoseObservation();

		this.inputs = new LimelightInputsSet();

		this.detectedObjectFilter = (rawDetection) -> true;
		this.mt1PoseFilter = Filter.nonFilteringFilter();
		this.mt2PoseFilter = Filter.nonFilteringFilter();

		this.calculateMT1StdDevs = () -> LimelightStdDevCalculations.DEFAULT_STD_DEVS;
		this.calculateMT2StdDevs = () -> LimelightStdDevCalculations.DEFAULT_STD_DEVS;

		setPipeline(pipeline);
	}

	public void updateObjectDetection() {
		if (pipeline.isDetectingObjects()) {
			detectedObjectObservations.clear();

			inputs.ObjectDetectionInputs().target2dValues = LimelightTarget2dValues.fromArray(LimelightHelpers.getT2DArray(name));
			inputs.ObjectDetectionInputs().rawDetections = LimelightHelpers.getRawDetections(name);
			Logger.processInputs(logPath + "/ObjectDetectionInputs", inputs.ObjectDetectionInputs());

			if (getTarget2dValues().isValid()) {
				for (LimelightHelpers.RawDetection rawDetection : getRawDetections()) {
					if (detectedObjectFilter.apply(rawDetection)) {
						pipeline.getDetectedObjectType(rawDetection.classId()).ifPresent(objectType -> {
							DetectedObjectObservation observation = ObjectDetectionMath.getDetectedObjectObservation(
								robotRelativeCameraPose,
								objectType,
								Rotation2d.fromDegrees(rawDetection.txnc()),
								Rotation2d.fromDegrees(rawDetection.tync()),
								getTarget2dTimestampSeconds(getTarget2dValues())
							);

							if (doesObservationExist(observation)) {
								detectedObjectObservations.add(observation);
							}
						});
					}
				}
				Logger
					.recordOutput(logPath + "/detectedObjectObservations", detectedObjectObservations.toArray(new DetectedObjectObservation[0]));
			}
		}
	}

	public void updateMT1() {
		if (pipeline.isUsingMT()) {
			inputs.mt1Inputs().mtRawData = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
			inputs.mt1Inputs().primaryTagPoseInCameraSpace = LimelightHelpers.getTargetPose3d_CameraSpace(name);
			Logger.processInputs(logPath + "/mt1Inputs", inputs.mt1Inputs());

			mt1PoseObservation = new RobotPoseObservation(getMT1RawData().timestampSeconds(), getMT1RawData().pose(), calculateMT1StdDevs.get());
			if (doesObservationExist(mt1PoseObservation)) {
				Logger.recordOutput(logPath + "/megaTag1PoseObservation", mt1PoseObservation);
			}
		}
	}

	public void updateMT2() {
		if (pipeline.isUsingMT()) {
			inputs.mt2Inputs().mtRawData = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
			inputs.mt2Inputs().primaryTagPoseInCameraSpace = LimelightHelpers.getTargetPose3d_CameraSpace(name);
			Logger.processInputs(logPath + "/mt2Inputs", inputs.mt2Inputs());

			mt2PoseObservation = new RobotPoseObservation(getMT2RawData().timestampSeconds(), getMT2RawData().pose(), calculateMT2StdDevs.get());
			if (doesObservationExist(mt2PoseObservation)) {
				Logger.recordOutput(logPath + "/megaTag2PoseObservation", mt2PoseObservation);
			}
		}
	}

	public String getName() {
		return name;
	}

	public Pose3d getRobotRelativeCameraPose() {
		return robotRelativeCameraPose;
	}

	@Override
	public List<DetectedObjectObservation> getRobotRelativeObjectTranslations() {
		if (pipeline.isDetectingObjects()) {
			return (ArrayList<DetectedObjectObservation>) detectedObjectObservations.clone();
		}
		return new ArrayList<>();
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

	public Function<LimelightHelpers.RawDetection, Boolean> getDetectedObjectFilter() {
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

	public void setDetectedObjectFilter(Function<LimelightHelpers.RawDetection, Boolean> detectedObjectFilter) {
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
		return inputs.ObjectDetectionInputs().target2dValues;
	}

	protected LimelightHelpers.RawDetection[] getRawDetections() {
		return inputs.ObjectDetectionInputs().rawDetections;
	}

	protected Pose3d getMT1PrimaryTagPoseInCameraSpace() {
		return inputs.mt1Inputs().primaryTagPoseInCameraSpace;
	}

	protected Pose3d getMT2PrimaryTagPoseInCameraSpace() {
		return inputs.mt2Inputs().primaryTagPoseInCameraSpace;
	}

	protected LimelightHelpers.PoseEstimate getMT1RawData() {
		return inputs.mt1Inputs().mtRawData;
	}

	protected LimelightHelpers.PoseEstimate getMT2RawData() {
		return inputs.mt2Inputs().mtRawData;
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

	private static double getTarget2dTimestampSeconds(LimelightTarget2dValues target2dValues) {
		return TimeUtil.getCurrentTimeSeconds()
			- Conversions.milliSecondsToSeconds(target2dValues.targetLatencyMilliseconds() + target2dValues.captureLatencyMilliseconds());
	}

	private static boolean doesObservationExist(DetectedObjectObservation detectedObjectObservation) {
		return !detectedObjectObservation.robotRelativeObjectTranslation().equals(Translation2d.kZero);
	}

	private static boolean doesObservationExist(RobotPoseObservation robotPoseObservation) {
		return !robotPoseObservation.robotPose().equals(Pose2d.kZero);
	}

}
