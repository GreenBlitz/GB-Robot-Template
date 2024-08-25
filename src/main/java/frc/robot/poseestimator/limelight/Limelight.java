package frc.robot.poseestimator.limelight;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.utils.GBSubsystem;

import java.util.Optional;

public class Limelight extends GBSubsystem {

    private NetworkTableEntry robotPoseEntry, idEntry, tagPoseEntry;

    private String name;

    public Limelight(String limelightName) {
        super("LimeLight " + limelightName + "\\");
        this.name = limelightName;
        String robotPoseQuery = "botpose_wpiblue";
        robotPoseEntry = NetworkTableInstance.getDefault().getTable(name).getEntry(robotPoseQuery);
        tagPoseEntry = NetworkTableInstance.getDefault().getTable(name).getEntry("targetpose_cameraspace");
        idEntry = NetworkTableInstance.getDefault().getTable(name).getEntry("tid");
    }

    public Optional<Pair<Pose2d, Double>> getUpdatedPose2DEstimation() {
        double[] poseArray = robotPoseEntry.getDoubleArray(new double[VisionConstants.LIMELIGHT_ENTRY_ARRAY_LENGTH]);
        double processingLatencySeconds = poseArray[VisionConstants.getValue(VisionConstants.LIMELIGHT_ARRAY_VALUES.TOTAL_LATENCY)] / 1000;
        double timestamp = Timer.getFPGATimestamp() - processingLatencySeconds;
        int id = (int) idEntry.getInteger(-1);

        Rotation2d angleOffset = AllianceUtilities.isBlueAlliance() ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(180);

        if (id == -1) {
            return Optional.empty();
        }

        Pose2d robotPose = new Pose2d(
                poseArray[VisionConstants.getValue(VisionConstants.LIMELIGHT_ARRAY_VALUES.X_AXIS)],
                poseArray[VisionConstants.getValue(VisionConstants.LIMELIGHT_ARRAY_VALUES.Y_AXIS)],
                Rotation2d.fromDegrees(poseArray[VisionConstants.getValue(VisionConstants.LIMELIGHT_ARRAY_VALUES.PITCH_ANGLE)] - angleOffset.getDegrees())
        );

        return Optional.of(new Pair<>(robotPose, timestamp));
    }

    public double getAprilTagHeight() {
        double[] poseArray = tagPoseEntry.getDoubleArray(new double[VisionConstants.LIMELIGHT_ENTRY_ARRAY_LENGTH]);
        return poseArray[VisionConstants.getValue(VisionConstants.LIMELIGHT_ARRAY_VALUES.Y_AXIS)];
    }

    public boolean getAprilTagConfidence() {
        boolean aprilTagHeightConfidence = getAprilTagHeight() < VisionConstants.APRIL_TAG_HEIGHT_METERS + VisionConstants.APRIL_TAG_HEIGHT_TOLERANCE_METERS
                || getAprilTagHeight() > VisionConstants.APRIL_TAG_HEIGHT_METERS - VisionConstants.APRIL_TAG_HEIGHT_TOLERANCE_METERS;
        return aprilTagHeightConfidence;
    }

    public double getDistanceFromAprilTag() {
        double[] poseArray = tagPoseEntry.getDoubleArray(new double[VisionConstants.LIMELIGHT_ENTRY_ARRAY_LENGTH]);
        return poseArray[VisionConstants.getValue(VisionConstants.LIMELIGHT_ARRAY_VALUES.Z_AXIS)];
    }

    public boolean hasTarget() {
        return getUpdatedPose2DEstimation().isPresent();
    }

    @Override
    protected void subsystemPeriodic() {

    }
}
