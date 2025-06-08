package frc.robot.vision.od;

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

import java.util.ArrayList;
import java.util.Optional;

public class LimeLightObjectDetector implements ObjectDetector {

	private final String logPath;
	private final String cameraNetworkTablesName;
	private final Pose3d cameraPose;

	private Filter<? super ObjectData> filter;
	private ArrayList<ObjectData> detectedObjects;
	private Optional<ObjectData> closestObject;

	private final NetworkTableEntry allObjectsEntry;
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

		allObjectsEntry = getLimelightNetworkTableEntry("rawdetections");
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

	private static ArrayList<ObjectData> objectsEntryToObjectDataArray(
		NetworkTableEntry allObjectsEntry,
		NetworkTableEntry pipelineLatencyEntry,
		NetworkTableEntry captureLatencyEntry,
		Pose3d cameraPose
	) {
		double[] entryArray = allObjectsEntry.getDoubleArray(new double[0]);
		int objectAmount = entryArray.length / VisionConstants.OBJECT_CELL_AMOUNT_IN_RAW_DETECTIONS_ENTRY;
		ArrayList<ObjectData> detectedObjects = new ArrayList<>();

		for (int i = 0; i < objectAmount; i++) {
			int firstCell = VisionConstants.OBJECT_CELL_AMOUNT_IN_RAW_DETECTIONS_ENTRY * i;
			ObjectData objectData = objectsEntryArrayToObjectData(entryArray, firstCell, pipelineLatencyEntry, captureLatencyEntry, cameraPose);
			detectedObjects.add(objectData);
		}
		return detectedObjects;
	}

	private static ObjectData objectsEntryArrayToObjectData(
		double[] entryArray,
		int firstCell,
		NetworkTableEntry pipelineLatencyEntry,
		NetworkTableEntry captureLatencyEntry,
		Pose3d cameraPose
	) {
		String objectType = "AAAAAAAAAAAA";

		Rotation2d cameraRelativeYaw = Rotation2d
			.fromDegrees(entryArray[firstCell + 1] - (VisionConstants.LIMELIGHT_OBJECT_DETECTION_HORIZONTAL_FOV.getDegrees() / 2));
		Rotation2d cameraRelativePitch = Rotation2d
			.fromDegrees(entryArray[firstCell + 2] - (VisionConstants.LIMELIGHT_OBJECT_DETECTION_VERTICAL_FOV.getDegrees() / 2));
		Translation2d robotRelativeObjectPose = ObjectDetectionMath
			.cameraRollRelativeYawAndPitchToRobotRelativePose(cameraRelativeYaw, cameraRelativePitch, cameraPose, 0.203);

		double totalLatency = pipelineLatencyEntry.getDouble(0) + captureLatencyEntry.getDouble(0);
		double timeStamp = TimeUtil.getCurrentTimeSeconds() - totalLatency;

		return new ObjectData(robotRelativeObjectPose, objectType, timeStamp);
	}

	private static ObjectData closestObjectEntriesToObjectData(
		NetworkTableEntry txEntry,
		NetworkTableEntry tyEntry,
		NetworkTableEntry classificationEntry,
		NetworkTableEntry pipelineLatencyEntry,
		NetworkTableEntry captureLatencyEntry,
		Pose3d cameraPose
	) {
		String objectType = classificationEntry.getString("none");
		double centerOfObjectHeightMeters = switch (objectType) {
			case "algae" -> ObjectHeights.ALGAE.objectHeightMeters / 2;
			case "coral" -> ObjectHeights.CORAL.objectHeightMeters / 2;
			default -> 0;
		};

		Rotation2d cameraRelativeYaw = Rotation2d.fromDegrees(txEntry.getDouble(0));
		Rotation2d cameraRelativePitch = Rotation2d.fromDegrees(tyEntry.getDouble(0));
		Translation2d robotRelativeObjectPose = ObjectDetectionMath
			.cameraRollRelativeYawAndPitchToRobotRelativePose(cameraRelativeYaw, cameraRelativePitch, cameraPose, centerOfObjectHeightMeters);

		double totalLatency = pipelineLatencyEntry.getDouble(0) + captureLatencyEntry.getDouble(0);
		double timeStamp = TimeUtil.getCurrentTimeSeconds() - totalLatency;

		return new ObjectData(robotRelativeObjectPose, objectType, timeStamp);
	}

	@Override
	public ArrayList<ObjectData> getAllObjectData() {
		return objectsEntryToObjectDataArray(allObjectsEntry, closestObjectPipelineLatencyEntry, closestObjectCaptureLatencyEntry, cameraPose);
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
//		detectedObjects = getAllFilteredObjectData();
		closestObject = getFilteredClosestObjectData();
//		closestObject = getClosestFilteredObjectData();

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
