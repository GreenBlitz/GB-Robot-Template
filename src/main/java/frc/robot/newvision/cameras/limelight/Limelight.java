package frc.robot.newvision.cameras.limelight;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.newvision.RobotPoseObservation;
import frc.robot.newvision.interfaces.OrientationRequiringRobotPoseSupplier;
import frc.robot.newvision.interfaces.IndependentRobotPoseSupplier;
import frc.utils.Conversions;
import frc.utils.LimelightHelpers;
import frc.utils.filter.Filter;
import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;
import java.util.function.Supplier;

public class Limelight implements IndependentRobotPoseSupplier, OrientationRequiringRobotPoseSupplier {

	private final String name;
	private final String logPath;
	private final Pose3d robotRelativeCameraPose;

	private final RobotPoseObservation mt1RobotPoseObservation;
	private final RobotPoseObservation mt2RobotPoseObservation;

	private LimelightHelpers.PoseEstimate mt1RobotPoseEstimate;
	private LimelightHelpers.PoseEstimate mt2RobotPoseEstimate;

	private Filter mt1RobotPoseFilter;
	private Filter mt2RobotPoseFilter;

	private Supplier<Matrix<N3, N1>> calculateMT1StdDevs;
	private Supplier<Matrix<N3, N1>> calculateMT2StdDevs;

	private LimelightPipeline pipeline;

	public Limelight(String name, String logPathPrefix, Pose3d robotRelativeCameraPose, LimelightPipeline pipeline) {
		this.name = name;
		this.logPath = logPathPrefix + "/" + name;

		this.robotRelativeCameraPose = robotRelativeCameraPose;
		setRobotRelativeCameraPose(robotRelativeCameraPose);

		this.mt1RobotPoseObservation = new RobotPoseObservation();
		this.mt2RobotPoseObservation = new RobotPoseObservation();

		this.mt1RobotPoseFilter = Filter.nonFilteringFilter();
		this.mt2RobotPoseFilter = Filter.nonFilteringFilter();

		this.calculateMT1StdDevs = () -> LimelightStdDevCalculations.DEFAULT_STD_DEVS;
		this.calculateMT2StdDevs = () -> LimelightStdDevCalculations.DEFAULT_STD_DEVS;

		setPipeline(pipeline);
	}

	public void log() {
		switch (pipeline) {
			case APRIL_TAG -> {
				if (isObservationPresent(mt1RobotPoseObservation)) {
					Logger.recordOutput(logPath + "/mt1Pose", mt1RobotPoseObservation.getRobotPose());
					Logger.recordOutput(logPath + "/mt1StdDevs", mt1RobotPoseObservation.getStdDevs().getData());
				}
				if (isObservationPresent(mt2RobotPoseObservation)) {
					Logger.recordOutput(logPath + "/mt2Pose", mt2RobotPoseObservation.getRobotPose());
					Logger.recordOutput(logPath + "/mt2StdDevs", mt2RobotPoseObservation.getStdDevs().getData());
				}
			}
			default -> {}
		}
	}

	public void updateMT1() {
		if (pipeline.equals(LimelightPipeline.APRIL_TAG)) {
			mt1RobotPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
			mt1RobotPoseObservation.setObservationValues(
				getEstimateTimestampSeconds(mt1RobotPoseEstimate),
				mt1RobotPoseEstimate.pose,
				calculateMT1StdDevs.get()
			);
		}
	}

	public void updateMT2() {
		if (pipeline.equals(LimelightPipeline.APRIL_TAG)) {
			mt2RobotPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
			mt2RobotPoseObservation.setObservationValues(
				getEstimateTimestampSeconds(mt2RobotPoseEstimate),
				mt2RobotPoseEstimate.pose,
				calculateMT2StdDevs.get()
			);
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
		if (
			pipeline.equals(LimelightPipeline.APRIL_TAG)
				&& isObservationPresent(mt1RobotPoseObservation)
				&& mt1RobotPoseFilter.isPassFilter()
		) {
			return Optional.of(mt1RobotPoseObservation);
		}
		return Optional.empty();
	}

	@Override
	public Optional<RobotPoseObservation> getOrientationRequiringRobotPose() {
		if (
			pipeline.equals(LimelightPipeline.APRIL_TAG)
				&& isObservationPresent(mt2RobotPoseObservation)
				&& mt2RobotPoseFilter.isPassFilter()
		) {
			return Optional.of(mt2RobotPoseObservation);
		}
		return Optional.empty();
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

	public void setMT1RobotPoseFilter(Filter mt1RobotPoseFilter) {
		this.mt1RobotPoseFilter = mt1RobotPoseFilter;
	}

	public void setMT2RobotPoseFilter(Filter mt2RobotPoseFilter) {
		this.mt2RobotPoseFilter = mt2RobotPoseFilter;
	}

	public void setMT1StdDevsCalculation(Supplier<Matrix<N3, N1>> calculateMT1StdDevs) {
		this.calculateMT1StdDevs = calculateMT1StdDevs;
	}

	public void setMT2StdDevsCalculation(Supplier<Matrix<N3, N1>> calculateMT2StdDevs) {
		this.calculateMT2StdDevs = calculateMT2StdDevs;
	}

	protected LimelightHelpers.PoseEstimate getMT1RobotPoseEstimate() {
		return mt1RobotPoseEstimate;
	}

	protected LimelightHelpers.PoseEstimate getMT2RobotPoseEstimate() {
		return mt2RobotPoseEstimate;
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

	private static boolean isObservationPresent(RobotPoseObservation robotPoseObservation) {
		return !robotPoseObservation.getRobotPose().equals(Pose2d.kZero);
	}

	protected static double getEstimateTimestampSeconds(LimelightHelpers.PoseEstimate poseEstimate) {
		if (poseEstimate.timestampSeconds == 0) {
			return 0;
		}
		return TimeUtil.getCurrentTimeSeconds() - Conversions.milliSecondsToSeconds(poseEstimate.latency);
	}

}
