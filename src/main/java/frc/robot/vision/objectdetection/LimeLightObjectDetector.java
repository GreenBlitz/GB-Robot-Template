package frc.robot.vision.objectdetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.vision.VisionConstants;
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

	private final NetworkTableEntry closestObjectTxEntry;
	private final NetworkTableEntry closestObjectTyEntry;
	private final NetworkTableEntry closestObjectNameEntry;
	private final NetworkTableEntry closestObjectPipelineLatencyEntry;
	private final NetworkTableEntry closestObjectCaptureLatencyEntry;

	public LimeLightObjectDetector(String logPath, String cameraNetworkTablesName, Pose3d cameraPose) {
		this.logPath = logPath;
		this.cameraNetworkTablesName = cameraNetworkTablesName;
		this.cameraPose = cameraPose;
		clearFilter();

		closestObjectTxEntry = getLimelightNetworkTableEntry("tx");
		closestObjectTyEntry = getLimelightNetworkTableEntry("ty");
		closestObjectNameEntry = getLimelightNetworkTableEntry("tdclass");
		closestObjectPipelineLatencyEntry = getLimelightNetworkTableEntry("tl");
		closestObjectCaptureLatencyEntry = getLimelightNetworkTableEntry("cl");
	}

	private NetworkTableEntry getLimelightNetworkTableEntry(String entryName) {
		return NetworkTableInstance.getDefault().getTable(cameraNetworkTablesName).getEntry(entryName);
	}

	private static Optional<ObjectType> getObjectType(NetworkTableEntry nameEntry) {
		String nameEntryValue = nameEntry.getString(VisionConstants.CLASSIFICATION_ENTRY_NO_OBJECT_VALUE);
		for (ObjectType type : ObjectType.values()) {
			if (type.getNameEntryValue().equals(nameEntryValue)) {
				return Optional.of(type);
			}
		}
		return Optional.empty();
	}

	private static ObjectData getClosestObjectData(
		NetworkTableEntry txEntry,
		NetworkTableEntry tyEntry,
		NetworkTableEntry pipelineLatencyEntry,
		NetworkTableEntry captureLatencyEntry,
		ObjectType objectType,
		Pose3d cameraPose
	) {
		double centerOfObjectHeightMeters = objectType.getObjectHeightMeters() / 2;

		Rotation2d cameraRelativeObjectYaw = Rotation2d.fromDegrees(txEntry.getDouble(0));
		Rotation2d cameraRelativeObjectPitch = Rotation2d.fromDegrees(tyEntry.getDouble(0));
		Translation2d robotRelativeObjectTranslation = ObjectDetectionMath
			.getRobotRelativeTranslation(cameraRelativeObjectYaw, cameraRelativeObjectPitch, cameraPose, centerOfObjectHeightMeters);

		double totalLatency = pipelineLatencyEntry.getDouble(0) + captureLatencyEntry.getDouble(0);
		double timeStamp = TimeUtil.getCurrentTimeSeconds() - totalLatency;

		return new ObjectData(robotRelativeObjectTranslation, objectType, timeStamp);
	}

	@Override
	public Optional<ObjectData> getClosestObjectData() {
		Optional<ObjectType> objectType = getObjectType(closestObjectNameEntry);

		if (objectType.isEmpty()) {
			return Optional.empty();
		}
		return Optional.of(
			getClosestObjectData(
				closestObjectTxEntry,
				closestObjectTyEntry,
				closestObjectPipelineLatencyEntry,
				closestObjectCaptureLatencyEntry,
				objectType.get(),
				cameraPose
			)
		);
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
