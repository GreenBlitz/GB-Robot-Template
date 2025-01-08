package frc.robot.vision.sources.limelight;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.GBSubsystem;
import frc.utils.Conversions;
import frc.utils.time.TimeUtils;

import java.util.Optional;

public class Limelight extends GBSubsystem {

    private final NetworkTableEntry robotPoseEntry;
    private final NetworkTableEntry aprilTagIdEntry;
    private final NetworkTableEntry aprilTagPoseEntry;
    private final NetworkTableEntry robotOrientationEntry;
    private final NetworkTableEntry robotPoseForHeadingEntry;
    private final String name;
    private double[] robotPoseArray;
    private double[] aprilTagPoseArray;
    private Rotation2d robotHeading;
    private GyroAngleValues gyroAngleValues;

    public Limelight(String name, String logPath) {
        super(logPath + name + "/");

        this.name = name;
        this.robotPoseEntry = getLimelightNetworkTableEntry("botpose_wpiblue");
        this.aprilTagPoseEntry = getLimelightNetworkTableEntry("targetpose_cameraspace");
        this.aprilTagIdEntry = getLimelightNetworkTableEntry("tid");
        this.robotOrientationEntry = getLimelightNetworkTableEntry("robot_orientation_set");
        this.robotPoseForHeadingEntry = getLimelightNetworkTableEntry("botpose_wpiblue");
        this.gyroAngleValues = new GyroAngleValues(0, 0, 0, 0, 0, 0);
    }

    public void updateGyroAngleValues(GyroAngleValues gyroAngleValues) {
        this.gyroAngleValues = gyroAngleValues;
    }

    public void updateLimelight() {
        robotOrientationEntry.setDoubleArray(
                new double[] {
                        gyroAngleValues.yaw(),
                        gyroAngleValues.yawRate(),
                        gyroAngleValues.pitch(),
                        gyroAngleValues.pitchRate(),
                        gyroAngleValues.roll(),
                        gyroAngleValues.rollRate(),}
        );
        robotPoseArray = robotPoseEntry.getDoubleArray(new double[LimelightConstants.LIMELIGHT_ENTRY_ARRAY_LENGTH]);
        aprilTagPoseArray = aprilTagPoseEntry.getDoubleArray(new double[LimelightConstants.LIMELIGHT_ENTRY_ARRAY_LENGTH]);
        double[] robotPoseWithoutGyroInput = robotPoseForHeadingEntry
                .getDoubleArray(new double[LimelightConstants.LIMELIGHT_ENTRY_ARRAY_LENGTH]);
        robotHeading = Rotation2d.fromDegrees(robotPoseWithoutGyroInput[LimelightEntryValue.YAW_ANGLE.getIndex()]);
    }

    public Optional<Pair<Pose3d, Double>> getUpdatedPose3DEstimation() {
        int id = (int) aprilTagIdEntry.getInteger(LimelightConstants.NO_APRILTAG_ID);
        if (id == LimelightConstants.NO_APRILTAG_ID) {
            return Optional.empty();
        }

        double processingLatencySeconds = Conversions.millisecondsToSeconds(robotPoseArray[LimelightEntryValue.TOTAL_LATENCY.getIndex()]);
        double timestamp = TimeUtils.getCurrentTimeSeconds() - processingLatencySeconds;

        Pose3d robotPose = new Pose3d(
                getPoseValue(LimelightEntryValue.X_AXIS),
                getPoseValue(LimelightEntryValue.Y_AXIS),
                getPoseValue(LimelightEntryValue.Z_AXIS),
                new Rotation3d(
                        Math.toRadians(getPoseValue(LimelightEntryValue.ROLL_ANGLE)),
                        Math.toRadians(getPoseValue(LimelightEntryValue.PITCH_ANGLE)),
                        Math.toRadians(getPoseValue(LimelightEntryValue.YAW_ANGLE))
                )
        );
        return Optional.of(new Pair<>(robotPose, timestamp));
    }

    public double getAprilTagHeight() {
        return getAprilTagValue(LimelightEntryValue.Y_AXIS);
    }

    public double getDistanceFromAprilTag() {
        return getAprilTagValue(LimelightEntryValue.Z_AXIS);
    }

    public double getAprilTagValue(LimelightEntryValue entryValue) {
        return aprilTagPoseArray[entryValue.getIndex()];
    }

    public double getPoseValue(LimelightEntryValue entryValue) {
        return robotPoseArray[entryValue.getIndex()];
    }

    public Optional<Rotation2d> getRobotHeading() {
        int id = (int) aprilTagIdEntry.getInteger(LimelightConstants.NO_APRILTAG_ID);
        if (id == LimelightConstants.NO_APRILTAG_ID) {
            return Optional.empty();
        }
        return Optional.of(robotHeading);
    }

    private NetworkTableEntry getLimelightNetworkTableEntry(String entryName) {
        return NetworkTableInstance.getDefault().getTable(name).getEntry(entryName);
    }

}