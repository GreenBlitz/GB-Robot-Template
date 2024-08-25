package edu.greenblitz.robotName.subsystems.limelight;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.poseestimator.limelight.VisionConstants;
import frc.utils.GBSubsystem;

import java.util.Optional;

public class Limelight extends GBSubsystem {

	private NetworkTableEntry robotPoseEntry, idEntry, tagPoseEntry;

	private String name;

	public Limelight(String limelightName) {
        super(limelightName);
		this.name = limelightName;
		String robotPoseQuery =  "botpose_wpiblue";
		robotPoseEntry = NetworkTableInstance.getDefault().getTable(name).getEntry(robotPoseQuery);
		tagPoseEntry = NetworkTableInstance.getDefault().getTable(name).getEntry("targetpose_cameraspace");
		idEntry = NetworkTableInstance.getDefault().getTable(name).getEntry("tid");
	}

    @Override
    protected void subsystemPeriodic() {

    }

    public Optional<Pair<Pose2d, Double>> getUpdatedPose2DEstimation() {
		double[] poseArray = robotPoseEntry.getDoubleArray(new double[VisionConstants.LIMELIGHT_ENTRY_ARRAY_LENGTH]);
		double processingLatency = poseArray[VisionConstants.getValue(VisionConstants.LIMELIGHT_ARRAY_VALUES.TOTAL_LATENCY)] / 1000;
		double timestamp = Timer.getFPGATimestamp() - processingLatency;
		int id = (int) idEntry.getInteger(-1);
		double angleOffset = AllianceUtilities.isBlueAlliance() ? 0 : 180;
		if (id == -1) {
			return Optional.empty();
		}
		Pose2d robotPose = new Pose2d(
				poseArray[VisionConstants.getValue(VisionConstants.LIMELIGHT_ARRAY_VALUES.X_AXIS)],
				poseArray[VisionConstants.getValue(VisionConstants.LIMELIGHT_ARRAY_VALUES.Y_AXIS)],
				Rotation2d.fromDegrees(poseArray[VisionConstants.getValue(VisionConstants.LIMELIGHT_ARRAY_VALUES.PITCH_ANGLE)] - angleOffset)
		);
		return Optional.of(new Pair<>(robotPose, timestamp));
	}

	public double getTagHeight() {
		double[] poseArray = tagPoseEntry.getDoubleArray(new double[VisionConstants.LIMELIGHT_ENTRY_ARRAY_LENGTH]);
		return poseArray[VisionConstants.getValue(VisionConstants.LIMELIGHT_ARRAY_VALUES.Y_AXIS)];
	}

	public boolean getTagConfidence() {
		boolean tagHeightConfidence = getTagHeight() < VisionConstants.APRIL_TAG_HEIGHT_METERS + VisionConstants.APRIL_TAG_HEIGHT_TOLERANCE_METERS
				|| getTagHeight() > VisionConstants.APRIL_TAG_HEIGHT_METERS - VisionConstants.APRIL_TAG_HEIGHT_TOLERANCE_METERS;
		return tagHeightConfidence;
	}

	public double getDistanceFromTag() {
		double[] poseArray = tagPoseEntry.getDoubleArray(new double[VisionConstants.LIMELIGHT_ENTRY_ARRAY_LENGTH]);
		return poseArray[VisionConstants.getValue(VisionConstants.LIMELIGHT_ARRAY_VALUES.Z_AXIS)];
	}

	public boolean hasTarget() {
		return getUpdatedPose2DEstimation().isPresent();
	}
}