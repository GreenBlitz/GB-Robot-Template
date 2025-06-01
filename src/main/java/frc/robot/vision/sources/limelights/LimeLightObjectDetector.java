package frc.robot.vision.sources.limelights;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.vision.VisionConstants;
import frc.robot.vision.data.ObjectData;
import frc.robot.vision.sources.ObjectDetector;
import frc.utils.Filter;
import frc.utils.math.ObjectDetectionMath;

import java.util.ArrayList;
import java.util.Optional;

public class LimeLightObjectDetector implements ObjectDetector {

	private final String logPath;
	private final String cameraNetworkTablesName;
	private final String detectorName;
	private final Pose3d cameraPose;

	private Filter<? super ObjectData> filter;
	private ArrayList<ObjectData> detectedObjects;

	private final NetworkTableEntry allObjectsEntry;

	public LimeLightObjectDetector(String logPath, String cameraNetworkTablesName, String detectorName, Pose3d cameraPose) {
		this.logPath = logPath;
		this.cameraNetworkTablesName = cameraNetworkTablesName;
		this.detectorName = detectorName;
		this.cameraPose = cameraPose;

		allObjectsEntry = getLimelightNetworkTableEntry("rawdetections");
	}

	private ArrayList<ObjectData> objectsEntryToObjectDataArray(NetworkTableEntry allObjectsEntry, ArrayList<ObjectData> detectedObjects) {
		double[] entryArray = allObjectsEntry.getDoubleArray(new double[0]);
		int objectAmount = entryArray.length / VisionConstants.OBJECT_CELL_AMOUNT_IN_RAW_DETECTIONS_ENTRY;

		for (int i = 0; i < objectAmount; i++) {
			int firstCell = VisionConstants.OBJECT_CELL_AMOUNT_IN_RAW_DETECTIONS_ENTRY * i;
			ObjectData objectData = objectsEntryArrayToObjectData(entryArray, firstCell);
			detectedObjects.add(objectData);
		}
		return detectedObjects;
	}

	private ObjectData objectsEntryArrayToObjectData(double[] entryArray, int firstCell) {
		Rotation2d cameraRelativeYaw = Rotation2d.fromDegrees(entryArray[firstCell + 1]);
		Rotation2d cameraRelativePitch = Rotation2d.fromDegrees(entryArray[firstCell + 2]);
		double xAxisDistance = ObjectDetectionMath.getCameraRelativeXAxisDistance(cameraRelativePitch, cameraPose);
		double yAxisDistance = ObjectDetectionMath.getCameraRelativeYAxisDistance(cameraRelativeYaw, cameraPose, xAxisDistance);
		Translation2d cameraRelativeObjectPose = new Translation2d(xAxisDistance, yAxisDistance);

		String objectType = "AAAAAAAAAAAA";

		double timeStamp = 100;

		return new ObjectData(cameraRelativeObjectPose, objectType, timeStamp);
	}

	protected NetworkTableEntry getLimelightNetworkTableEntry(String entryName) {
		return NetworkTableInstance.getDefault().getTable(cameraNetworkTablesName).getEntry(entryName);
	}

	@Override
	public void update() {}

	@Override
	public ArrayList<ObjectData> getAllObjectData() {
		return null;
	}

	@Override
	public Optional<ObjectData> getClosestObjectData() {
		return Optional.empty();
	}

	@Override
	public Optional<ObjectData> getFilteredClosestObjectData() {
		return Optional.empty();
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
