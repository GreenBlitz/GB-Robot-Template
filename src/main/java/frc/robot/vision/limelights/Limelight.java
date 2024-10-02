package frc.robot.vision.limelights;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.utils.GBSubsystem;

import java.util.Optional;

public class Limelight extends GBSubsystem {

	private final NetworkTableEntry robotPoseEntry;
	private final NetworkTableEntry aprilTagIdEntry;
	private final NetworkTableEntry aprilTagPoseEntry;
	private final NetworkTableEntry robotOrientationEntry;
	private final String name;
	private double[] robotPoseArray;
	private double[] aprilTagPoseArray;
	private GyroAngleValues gyroAngleValues;

	public Limelight(String name, String hardwareLogPath) {
		super(hardwareLogPath + name + "/");

		this.name = name;
		this.robotPoseEntry = getLimelightNetworkTableEntry("botpose_orb_wpiblue");
		this.aprilTagPoseEntry = getLimelightNetworkTableEntry("targetpose_cameraspace");
		this.aprilTagIdEntry = getLimelightNetworkTableEntry("tid");
		this.robotOrientationEntry = getLimelightNetworkTableEntry("robot_orientation_set");
		this.gyroAngleValues = new GyroAngleValues(0, 0, 0, 0, 0, 0);
	}

	public void updateGyroAngleValues(GyroAngleValues gyroAngleValues) {
		this.gyroAngleValues = gyroAngleValues;
	}

	public void updateLimelight() {
		robotOrientationEntry.setDoubleArray(gyroAngleValues.getAsArray());
		robotPoseArray = robotPoseEntry.getDoubleArray(new double[LimeLightConstants.LIMELIGHT_ENTRY_ARRAY_LENGTH]);
		aprilTagPoseArray = aprilTagPoseEntry.getDoubleArray(new double[LimeLightConstants.LIMELIGHT_ENTRY_ARRAY_LENGTH]);
	}

	public Optional<Pair<Pose3d, Double>> getUpdatedPose3DEstimation() {
		int id = (int) aprilTagIdEntry.getInteger(-1);
		if (id == -1) {
			return Optional.empty();
		}

		double processingLatencySeconds = robotPoseArray[LimelightEntryValue.TOTAL_LATENCY.getIndex()] / 1000;
		double timestamp = Timer.getFPGATimestamp() - processingLatencySeconds;

		Pose3d robotPose = new Pose3d(
			getPoseInformation(LimelightEntryValue.X_AXIS),
			getPoseInformation(LimelightEntryValue.Y_AXIS),
			getPoseInformation(LimelightEntryValue.Z_AXIS),
			new Rotation3d(
				Math.toRadians(getPoseInformation(LimelightEntryValue.ROLL_ANGLE)),
				Math.toRadians(getPoseInformation(LimelightEntryValue.PITCH_ANGLE)),
				Math.toRadians(getPoseInformation(LimelightEntryValue.YAW_ANGLE))
			)
		);
		return Optional.of(new Pair<>(robotPose, timestamp));
	}

	public double getAprilTagHeight() {
		return getAprilTagInformation(LimelightEntryValue.Y_AXIS);
	}

	public double getDistanceFromAprilTag() {
		return getAprilTagInformation(LimelightEntryValue.Z_AXIS);
	}

	public double getAprilTagInformation(LimelightEntryValue entryValue) {
		return aprilTagPoseArray[entryValue.getIndex()];
	}

	public double getPoseInformation(LimelightEntryValue entryValue) {
		return robotPoseArray[entryValue.getIndex()];
	}

	private NetworkTableEntry getLimelightNetworkTableEntry(String entryName) {
		return NetworkTableInstance.getDefault().getTable(name).getEntry(entryName);
	}

	@Override
	protected void subsystemPeriodic() {}

}
