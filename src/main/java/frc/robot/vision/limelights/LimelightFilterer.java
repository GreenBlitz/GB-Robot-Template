package frc.robot.vision.limelights;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.poseestimator.GBPoseEstimator;
import frc.robot.poseestimator.IPoseEstimator;
import frc.robot.poseestimator.PoseArrayEntryValue;
import frc.robot.poseestimator.PoseEstimationMath;
import frc.robot.poseestimator.observations.VisionObservation;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class LimelightFilterer extends GBSubsystem {

	private final MultiLimelightsRawData multiLimelightsRawData;
	private final IPoseEstimator poseEstimator;
	private final LinearFilter linearFilterX;
	private final LinearFilter linearFilterY;
//	private final TimeInterpolatableBuffer<Pose2d> limelightPoseInterpolator;

	public LimelightFilterer(LimelightFiltererConfig config, IPoseEstimator poseEstimator) {
		super(config.logPath());

		this.multiLimelightsRawData = new MultiLimelightsRawData(config.limelightsNames(), config.hardwareLogPath());
		this.poseEstimator = poseEstimator;
		this.linearFilterX = LinearFilter.movingAverage(20);
		this.linearFilterY = LinearFilter.movingAverage(20);
//		this.limelightPoseInterpolator = TimeInterpolatableBuffer.createBuffer(PoseEstimatorConstants.POSE_BUFFER_SIZE_SECONDS);
	}

	public void updateGyroAngles(GyroAngleValues gyroAnglesValues) {
		multiLimelightsRawData.updateGyroAngles(gyroAnglesValues);
	}

	public List<VisionObservation> getFilteredVisionObservations() {
		ArrayList<VisionObservation> estimates = new ArrayList<>();

		for (LimelightRawData limelightRawData : multiLimelightsRawData.getAllAvailableLimelightData()) {
			if (keepLimelightData(limelightRawData)) {
				estimates.add(rawDataToObservation(limelightRawData));
			}
		}

		return estimates;
	}

	public List<VisionObservation> getAllAvailableLimelightData() {
		ArrayList<VisionObservation> estimates = new ArrayList<>();

		for (LimelightRawData limelightRawData : multiLimelightsRawData.getAllAvailableLimelightData()) {
			estimates.add(rawDataToObservation(limelightRawData));
		}

		return estimates;
	}

	private VisionObservation rawDataToObservation(LimelightRawData limelightRawData) {
		Optional<Pose2d> estimatedPoseAtTimeStamp = poseEstimator.getEstimatedPoseAtTimeStamp(limelightRawData.timestamp());
		double[] standardTransformDeviations;
		standardTransformDeviations = estimatedPoseAtTimeStamp
			.map(pose2d -> PoseEstimationMath.calculateStandardDeviationOfPose(limelightRawData, pose2d))
			.orElseGet(() -> PoseEstimationMath.calculateStandardDeviationOfPose(limelightRawData, poseEstimator.getEstimatedPose()));
		double[] standardDeviations = new double[] {
			standardTransformDeviations[PoseArrayEntryValue.X_VALUE.getEntryValue()],
			standardTransformDeviations[PoseArrayEntryValue.Y_VALUE.getEntryValue()],
			LimeLightConstants.STANDARD_DEVIATION_VISION_ANGLE};

		return new VisionObservation(limelightRawData.estimatedPose().toPose2d(), standardDeviations, limelightRawData.timestamp());
	}

	private boolean keepLimelightData(LimelightRawData limelightRawData) {
		return LimelightFilters.isAprilTagInProperHeight(limelightRawData)
			&& LimelightFilters.isLimelightOutputInTolerance(limelightRawData, poseEstimator.getEstimatedPose())
			&& LimelightFilters.isRollZero(limelightRawData)
			&& LimelightFilters.isPitchZero(limelightRawData)
			&& LimelightFilters.isRobotOnGround(limelightRawData);
	}

	public void logEstimatedPositions() {
		List<VisionObservation> observations = getFilteredVisionObservations();

		for (int i = 0; i < observations.size(); i++) {
			Logger.recordOutput(
				super.getLogPath() + LimeLightConstants.ESTIMATION_LOGPATH_PREFIX + i + "Time" + observations.get(i).timestamp(),
				observations.get(i).visionPose()
			);
		}
	}

	@Override
	protected void subsystemPeriodic() {
		for (VisionObservation observation : getFilteredVisionObservations()) {
			linearFilterX.calculate(observation.visionPose().getX());
			linearFilterY.calculate(observation.visionPose().getY());
		}
		Pose2d visionPose = new Pose2d(linearFilterX.lastValue(), linearFilterY.lastValue(), poseEstimator.getEstimatedPose().getRotation());
		Pose2d estimatedPose = poseEstimator.getEstimatedPose();
		Pose3d limelightPosition = new Pose3d(
			visionPose.getX(),
			visionPose.getY(),
			0,
			new Rotation3d(0, 0, visionPose.getRotation().getRadians())
		);
		Pose3d estimatedPose3d = new Pose3d(
			estimatedPose.getX(),
			estimatedPose.getY(),
			0,
			new Rotation3d(0, 0, estimatedPose.getRotation().getRadians())
		);
		Transform3d transformDifference = limelightPosition.minus(estimatedPose3d);
		Rotation3d rotationDifference = limelightPosition.getRotation().minus(estimatedPose3d.getRotation());
		boolean is = transformDifference.getTranslation().getNorm() <= LimeLightConstants.POSITION_NORM_TOLERANCE
			&& LimelightFilters.getRotationNorm(rotationDifference) <= LimeLightConstants.ROTATION_NORM_TOLERANCE;

		if (!is) {
			((GBPoseEstimator) poseEstimator).resetPoseByLimelight();
			linearFilterX.reset();
			linearFilterY.reset();
		}
	}

}
