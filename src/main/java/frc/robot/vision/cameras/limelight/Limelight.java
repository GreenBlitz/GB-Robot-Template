package frc.robot.vision.cameras.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.vision.RobotPoseObservation;
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

public class Limelight implements IndependentRobotPoseSupplier, OrientationRequiringRobotPoseSupplier {

	private final String name;
	private final String logPath;
	private final Pose3d robotRelativeCameraPose;

	private RobotPoseObservation mt1PoseObservation;
	private RobotPoseObservation mt2PoseObservation;

	private LimelightHelpers.PoseEstimate mt1RawData;
	private LimelightHelpers.PoseEstimate mt2RawData;

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

		this.mt1PoseObservation = new RobotPoseObservation();
		this.mt2PoseObservation = new RobotPoseObservation();

		this.mt1PoseFilter = Filter.nonFilteringFilter();
		this.mt2PoseFilter = Filter.nonFilteringFilter();

		this.calculateMT1StdDevs = () -> LimelightStdDevCalculations.DEFAULT_STD_DEVS;
		this.calculateMT2StdDevs = () -> LimelightStdDevCalculations.DEFAULT_STD_DEVS;

		setPipeline(pipeline);
	}

	public void log() {
		switch (pipeline) {
			case APRIL_TAG -> {
				if (doesObservationExist(mt1PoseObservation)) {
					Logger.recordOutput(logPath + "/megaTag1PoseObservation", mt1PoseObservation);
				}
				if (doesObservationExist(mt2PoseObservation)) {
					Logger.recordOutput(logPath + "/megaTag2PoseObservation", mt2PoseObservation);
				}
			}
			default -> {}
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

	private static boolean doesObservationExist(RobotPoseObservation robotPoseObservation) {
		return !robotPoseObservation.robotPose().equals(Pose2d.kZero);
	}

	protected static double getEstimateTimestampSeconds(LimelightHelpers.PoseEstimate poseEstimate) {
		if (poseEstimate.timestampSeconds == 0) {
			return 0;
		}
		return TimeUtil.getCurrentTimeSeconds() - Conversions.milliSecondsToSeconds(poseEstimate.latency);
	}

}
