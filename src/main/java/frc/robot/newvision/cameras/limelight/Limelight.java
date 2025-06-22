package frc.robot.newvision.cameras.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
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

	private final Pose2d megaTag1MinStandardDeviations;
	private final Pose2d megaTag1StandardDeviationFactors;
	private final Pose2d megaTag2MinStandardDeviations;
	private final Pose2d megaTag2StandardDeviationFactors;

	private final RobotPoseObservation megaTag1RobotPoseObservation;
	private final RobotPoseObservation megaTag2RobotPoseObservation;

	private LimelightPipeline pipeline;

	public Limelight(
		String name,
		String logPathPrefix,
		Pose3d robotRelativeCameraPose,
		LimelightPipeline pipeline,
		Pose2d megaTag1MinStandardDeviations,
		Pose2d megaTag1StandardDeviationFactors,
		Pose2d megaTag2MinStandardDeviations,
		Pose2d megaTag2StandardDeviationFactors
	) {
		this.name = name;
		this.logPath = logPathPrefix + "/" + name;

		this.robotRelativeCameraPose = robotRelativeCameraPose;
		setRobotRelativeCameraPose(robotRelativeCameraPose);

		this.megaTag1MinStandardDeviations = megaTag1MinStandardDeviations;
		this.megaTag1StandardDeviationFactors = megaTag1StandardDeviationFactors;
		this.megaTag2MinStandardDeviations = megaTag2MinStandardDeviations;
		this.megaTag2StandardDeviationFactors = megaTag2StandardDeviationFactors;

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
					calculateStandardDeviations(megaTag1RobotPoseEstimate, megaTag1MinStandardDeviations, megaTag1StandardDeviationFactors)
				);
				megaTag2RobotPoseObservation.setObservationValues(
					getEstimateTimestampSeconds(megaTag2RobotPoseEstimate),
					megaTag2RobotPoseEstimate.pose,
					calculateStandardDeviations(megaTag2RobotPoseEstimate, megaTag2MinStandardDeviations, megaTag2StandardDeviationFactors)
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

	private static Pose2d calculateStandardDeviations(LimelightHelpers.PoseEstimate poseEstimate, Pose2d minStandardDeviations, Pose2d standardDeviationFactors) {
		double averageTagDistanceSquared =  Math.pow(poseEstimate.avgTagDist, 2);
		return new Pose2d(
				Math.max(minStandardDeviations.getX(), standardDeviationFactors.getX() * averageTagDistanceSquared),
				Math.max(minStandardDeviations.getY(), standardDeviationFactors.getY() * averageTagDistanceSquared),
				new Rotation2d(Math.max(minStandardDeviations.getRotation().getRadians(), standardDeviationFactors.getRotation().times(averageTagDistanceSquared).getRadians()))
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
