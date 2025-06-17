package frc.robot.vision.od;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.vision.data.ObjectData;
import frc.utils.Filter;
import frc.utils.math.ObjectDetectionMath;
import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

public class LimeLightObjectDetector implements ObjectDetector {

	private final String logPath;
	private final String cameraNetworkTablesName;
	private final Pose3d cameraPose;

	private Filter<? super ObjectData> filter;
	private Optional<ObjectData> closestObject;

	private final NetworkTableEntry doesTargetExistEntry;
	private final NetworkTableEntry closestObjectTxEntry;
	private final NetworkTableEntry closestObjectTyEntry;
	private final NetworkTableEntry closestObjectClassificationEntry;
	private final NetworkTableEntry closestObjectPipelineLatencyEntry;
	private final NetworkTableEntry closestObjectCaptureLatencyEntry;

	public LimeLightObjectDetector(String logPath, String cameraNetworkTablesName, Pose3d cameraPose) {
		this.logPath = logPath;
		this.cameraNetworkTablesName = cameraNetworkTablesName;
		this.cameraPose = cameraPose;
		clearFilter();

		doesTargetExistEntry = getLimelightNetworkTableEntry("tv");
		closestObjectTxEntry = getLimelightNetworkTableEntry("tx");
		closestObjectTyEntry = getLimelightNetworkTableEntry("ty");
		closestObjectClassificationEntry = getLimelightNetworkTableEntry("tdclass");
		closestObjectPipelineLatencyEntry = getLimelightNetworkTableEntry("tl");
		closestObjectCaptureLatencyEntry = getLimelightNetworkTableEntry("cl");
	}

	protected NetworkTableEntry getLimelightNetworkTableEntry(String entryName) {
		return NetworkTableInstance.getDefault().getTable(cameraNetworkTablesName).getEntry(entryName);
	}

	private static ObjectData closestObjectEntriesToObjectData(
		NetworkTableEntry txEntry,
		NetworkTableEntry tyEntry,
		NetworkTableEntry classificationEntry,
		NetworkTableEntry pipelineLatencyEntry,
		NetworkTableEntry captureLatencyEntry,
		Pose3d cameraPose
	) {
		ObjectType objectType = switch (classificationEntry.getString("none")) {
			case "algae\r" -> ObjectType.ALGAE;
			case "coral" -> ObjectType.CORAL;
			default -> null;
		};
		double centerOfObjectHeightMeters = objectType.objectHeightMeters / 2;


		Rotation2d cameraRelativeYaw = Rotation2d.fromDegrees(txEntry.getDouble(0));
		Rotation2d cameraRelativePitch = Rotation2d.fromDegrees(tyEntry.getDouble(0));
		Translation2d robotRelativeObjectPose = ObjectDetectionMath
			.cameraRollRelativeYawAndPitchToRobotRelativePose(cameraRelativeYaw, cameraRelativePitch, cameraPose, centerOfObjectHeightMeters);

		double totalLatency = pipelineLatencyEntry.getDouble(0) + captureLatencyEntry.getDouble(0);
		double timeStamp = TimeUtil.getCurrentTimeSeconds() - totalLatency;

		return new ObjectData(robotRelativeObjectPose, objectType, timeStamp);
	}

	@Override
	public Optional<ObjectData> getClosestObjectData() {
		if (doesTargetExistEntry.getInteger(0) == 1) {
			return Optional.of(
				closestObjectEntriesToObjectData(
					closestObjectTxEntry,
					closestObjectTyEntry,
					closestObjectClassificationEntry,
					closestObjectPipelineLatencyEntry,
					closestObjectCaptureLatencyEntry,
					cameraPose
				)
			);
		}
		return Optional.empty();
	}

	@Override
	public void log() {
		if (closestObject.isPresent()) {
			Logger.recordOutput(
				logPath + "closestObjectTranslation",
				new Pose2d(closestObject.get().getRobotRelativeEstimatedTranslation(), new Rotation2d())
			);
		}
	}

	@Override
	public void update() {
		closestObject = getFilteredClosestObjectData();

		log();
	}

	@Override
	public void setFilter(Filter<? super ObjectData> newFilter) {
		this.filter = newFilter;
	}

	@Override
	public Filter<? super ObjectData> getFilter() {
		return filter;
	}

}
