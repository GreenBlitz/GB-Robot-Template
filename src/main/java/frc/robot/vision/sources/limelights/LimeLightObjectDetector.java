package frc.robot.vision.sources.limelights;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.vision.VisionConstants;
import frc.robot.vision.data.ObjectData;
import frc.robot.vision.sources.ObjectDetector;
import frc.utils.Filter;
import java.util.ArrayList;
import java.util.Optional;

public class LimeLightObjectDetector implements ObjectDetector {

	private final String logPath;
	private final String cameraNetworkTablesName;
	private final String detectorName;
	private Filter<? super ObjectData> filter;
	private ArrayList<ObjectData> allObjectData

	private final NetworkTableEntry allObjectsEntry;

	public LimeLightObjectDetector(String logPath, String cameraNetworkTablesName, String detectorName) {
		this.logPath = logPath;
		this.cameraNetworkTablesName = cameraNetworkTablesName;
		this.detectorName = detectorName;

		allObjectsEntry = getLimelightNetworkTableEntry("rawdetections");
	}

	private ArrayList<ObjectData> allObjectsEntryToObjectDataArray(NetworkTableEntry allObjectsEntry) {
		double[] entryArray = allObjectsEntry.getDoubleArray(new double[0]);
		int objectAmount = entryArray.length / VisionConstants.OBJECT_CELL_AMOUNT_IN_RAW_DETECTIONS_ENTRY;
	}

	protected NetworkTableEntry getLimelightNetworkTableEntry(String entryName) {
		return NetworkTableInstance.getDefault().getTable(cameraNetworkTablesName).getEntry(entryName);
	}

	@Override
	public void update() {

	}

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
