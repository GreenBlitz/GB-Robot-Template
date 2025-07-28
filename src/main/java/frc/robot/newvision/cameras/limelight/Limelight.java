package frc.robot.newvision.cameras.limelight;

import edu.wpi.first.math.Matrix;
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

	private final RobotPoseObservation megaTag1RobotPoseObservation;
	private final RobotPoseObservation megaTag2RobotPoseObservation;

	private LimelightHelpers.PoseEstimate megaTag1RobotPoseEstimate;
	private LimelightHelpers.PoseEstimate megaTag2RobotPoseEstimate;

	private Filter megaTag1RobotPoseFilter;
	private Filter megaTag2RobotPoseFilter;

	private Supplier<Matrix<N3, N1>> calculateMegaTag1StandardDeviations;
	private Supplier<Matrix<N3, N1>> calculateMegaTag2StandardDeviations;

	private LimelightPipeline pipeline;

	public Limelight(String name, String logPathPrefix, Pose3d robotRelativeCameraPose, LimelightPipeline pipeline) {
		this.name = name;
		this.logPath = logPathPrefix + "/" + name;

		this.robotRelativeCameraPose = robotRelativeCameraPose;
		setRobotRelativeCameraPose(robotRelativeCameraPose);

		this.megaTag1RobotPoseObservation = new RobotPoseObservation();
		this.megaTag2RobotPoseObservation = new RobotPoseObservation();

		this.megaTag1RobotPoseFilter = Filter.nonFilteringFilter();
		this.megaTag2RobotPoseFilter = Filter.nonFilteringFilter();

		this.calculateMegaTag1StandardDeviations = () -> LimelightStandardDeviationsCalculations.DEFAULT_STANDARD_DEVIATIONS;
		this.calculateMegaTag2StandardDeviations = () -> LimelightStandardDeviationsCalculations.DEFAULT_STANDARD_DEVIATIONS;

		setPipeline(pipeline);
	}

	public void log() {
		switch (pipeline) {
			case APRIL_TAG -> {
				if (isObservationPresent(megaTag1RobotPoseObservation)) {
					Logger.recordOutput(logPath + "/megaTag1Pose", megaTag1RobotPoseObservation.getRobotPose());
					Logger.recordOutput(logPath + "/megaTag1StandardDeviations", megaTag1RobotPoseObservation.getStandardDeviations());
				}
				if (isObservationPresent(megaTag2RobotPoseObservation)) {
					Logger.recordOutput(logPath + "/megaTag2Pose", megaTag2RobotPoseObservation.getRobotPose());
					Logger.recordOutput(logPath + "/megaTag2StandardDeviations", megaTag2RobotPoseObservation.getStandardDeviations());
				}
			}
			default -> {}
		}
	}

	public void updateMegaTag1() {
		if (pipeline.equals(LimelightPipeline.APRIL_TAG)) {
			megaTag1RobotPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
			megaTag1RobotPoseObservation.setObservationValues(
				getEstimateTimestampSeconds(megaTag1RobotPoseEstimate),
				megaTag1RobotPoseEstimate.pose,
				calculateMegaTag1StandardDeviations.get()
			);
		}
	}

	public void updateMegaTag2() {
		if (pipeline.equals(LimelightPipeline.APRIL_TAG)) {
			megaTag2RobotPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
			megaTag2RobotPoseObservation.setObservationValues(
				getEstimateTimestampSeconds(megaTag2RobotPoseEstimate),
				megaTag2RobotPoseEstimate.pose,
				calculateMegaTag2StandardDeviations.get()
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
				&& isObservationPresent(megaTag1RobotPoseObservation)
				&& megaTag1RobotPoseFilter.isPassFilter()
		) {
			return Optional.of(megaTag1RobotPoseObservation);
		}
		return Optional.empty();
	}

	@Override
	public Optional<RobotPoseObservation> getOrientationRequiringRobotPose() {
		if (
			pipeline.equals(LimelightPipeline.APRIL_TAG)
				&& isObservationPresent(megaTag2RobotPoseObservation)
				&& megaTag2RobotPoseFilter.isPassFilter()
		) {
			return Optional.of(megaTag2RobotPoseObservation);
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

	public void setMegaTag1RobotPoseFilter(Filter megaTag1RobotPoseFilter) {
		this.megaTag1RobotPoseFilter = megaTag1RobotPoseFilter;
	}

	public void setMegaTag2RobotPoseFilter(Filter megaTag2RobotPoseFilter) {
		this.megaTag2RobotPoseFilter = megaTag2RobotPoseFilter;
	}

	public void setMegaTag1StandardDeviationsCalculationFunction(Supplier<Matrix<N3, N1>> calculateMegaTag1StandardDeviations) {
		this.calculateMegaTag1StandardDeviations = calculateMegaTag1StandardDeviations;
	}

	public void setMegaTag2StandardDeviationsCalculationFunction(Supplier<Matrix<N3, N1>> calculateMegaTag2StandardDeviations) {
		this.calculateMegaTag2StandardDeviations = calculateMegaTag2StandardDeviations;
	}

	protected LimelightHelpers.PoseEstimate getMegaTag1RobotPoseEstimate() {
		return megaTag1RobotPoseEstimate;
	}

	protected LimelightHelpers.PoseEstimate getMegaTag2RobotPoseEstimate() {
		return megaTag2RobotPoseEstimate;
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
		return robotPoseObservation.getTimestampSeconds() != 0;
	}

	private static double getEstimateTimestampSeconds(LimelightHelpers.PoseEstimate poseEstimate) {
		if (poseEstimate.timestampSeconds == 0) {
			return 0;
		}
		return TimeUtil.getCurrentTimeSeconds() - Conversions.milliSecondsToSeconds(poseEstimate.latency);
	}

}
