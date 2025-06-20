package frc.robot.newvision.cameras.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.newvision.RobotPoseObservation;
import frc.robot.newvision.interfaces.OrientationRequiringRobotPoseSupplier;
import frc.robot.newvision.interfaces.IndependentRobotPoseSupplier;
import frc.utils.Conversions;
import frc.utils.LimelightHelpers;
import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;
import java.util.function.Function;

public class Limelight implements IndependentRobotPoseSupplier, OrientationRequiringRobotPoseSupplier {

	private final String name;
	private final String logPath;
	private final Pose3d robotRelativeCameraPose;

	private final Function<LimelightHelpers.PoseEstimate, Pose2d> calculateMegaTag1StandardDeviations;
	private final Function<LimelightHelpers.PoseEstimate, Pose2d> calculateMegaTag2StandardDeviations;

	private final RobotPoseObservation megaTag1RobotPoseObservation;
	private final RobotPoseObservation megaTag2RobotPoseObservation;

	private LimelightPipeline pipeline;

	public Limelight(
		String name,
		String logPathPrefix,
		Pose3d robotRelativeCameraPose,
		LimelightPipeline pipeline,
		Function<LimelightHelpers.PoseEstimate, Pose2d> calculateMegaTag1StandardDeviations,
		Function<LimelightHelpers.PoseEstimate, Pose2d> calculateMegaTag2StandardDeviations
	) {
		this.name = name;
		this.logPath = logPathPrefix + "/" + name;

		this.robotRelativeCameraPose = robotRelativeCameraPose;
		setRobotRelativeCameraPose(robotRelativeCameraPose);

		this.calculateMegaTag1StandardDeviations = calculateMegaTag1StandardDeviations;
		this.calculateMegaTag2StandardDeviations = calculateMegaTag2StandardDeviations;

		this.megaTag1RobotPoseObservation = new RobotPoseObservation();
		this.megaTag2RobotPoseObservation = new RobotPoseObservation();

		setPipeline(pipeline);
	}

	public void update() {
		switch (pipeline) {
			case APRIL_TAG -> {
				LimelightHelpers.PoseEstimate megaTag1RobotPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
				LimelightHelpers.PoseEstimate megaTag2RobotPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

				megaTag1RobotPoseObservation.setObservationValues(
					getEstimateTimestampSeconds(megaTag1RobotPoseEstimate),
					megaTag1RobotPoseEstimate.pose,
					calculateMegaTag1StandardDeviations.apply(megaTag1RobotPoseEstimate)
				);
				megaTag2RobotPoseObservation.setObservationValues(
					getEstimateTimestampSeconds(megaTag2RobotPoseEstimate),
					megaTag2RobotPoseEstimate.pose,
					calculateMegaTag2StandardDeviations.apply(megaTag2RobotPoseEstimate)
				);
			}
			default -> {}
		}

		log();
	}

	private void log() {
		switch (pipeline) {
			case APRIL_TAG -> {
				Logger.recordOutput(logPath + "/megaTag1Pose", megaTag1RobotPoseObservation.getRobotPose());
				Logger.recordOutput(logPath + "/megaTag1StandardDeviations", megaTag1RobotPoseObservation.getStandardDeviations());
				Logger.recordOutput(logPath + "/megaTag2Pose", megaTag2RobotPoseObservation.getRobotPose());
				Logger.recordOutput(logPath + "/megaTag2StandardDeviations", megaTag2RobotPoseObservation.getStandardDeviations());
			}
			default -> {}
		}
	}

	public String getName() {
		return name;
	}

	public Pose3d getRobotRelativeCameraPose() {
		return robotRelativeCameraPose;
	}

	@Override
	public Optional<RobotPoseObservation> getRobotPose() {
		if (pipeline.equals(LimelightPipeline.APRIL_TAG) && isObservationValid(megaTag1RobotPoseObservation)) {
			return Optional.of(megaTag1RobotPoseObservation);
		}
		return Optional.empty();
	}

	@Override
	public Optional<RobotPoseObservation> getOrientationRequiringRobotPose() {
		if (pipeline.equals(LimelightPipeline.APRIL_TAG) && isObservationValid(megaTag2RobotPoseObservation)) {
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

	private void setRobotRelativeCameraPose(Pose3d robotRelativeCameraPose) {
		LimelightHelpers.setCameraPose_RobotSpace(
			name,
			robotRelativeCameraPose.getX(),
			robotRelativeCameraPose.getY(),
			robotRelativeCameraPose.getZ(),
			robotRelativeCameraPose.getRotation().getX(),
			robotRelativeCameraPose.getRotation().getY(),
			robotRelativeCameraPose.getRotation().getZ()
		);
	}

	private static boolean isObservationValid(RobotPoseObservation robotPoseObservation) {
		return robotPoseObservation.getTimestampSeconds() != 0;
	}

	private static double getEstimateTimestampSeconds(LimelightHelpers.PoseEstimate poseEstimate) {
		if (poseEstimate.timestampSeconds != 0) {
			return TimeUtil.getCurrentTimeSeconds() - Conversions.milliSecondsToSeconds(poseEstimate.latency);
		}
		return 0;
	}

}
