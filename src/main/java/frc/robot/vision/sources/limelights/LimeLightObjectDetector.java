package frc.robot.vision.sources.limelights;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.vision.data.ObjectData;
import frc.robot.vision.sources.ObjectDetector;
import frc.utils.Filter;
import java.util.ArrayList;
import java.util.Optional;

public class LimeLightObjectDetector implements ObjectDetector {

    private final String logPath;
    private final String cameraNetworkTablesName;
    private final String detectorName;
    private Filter<ObjectData> filter;

    public LimeLightObjectDetector(String logPath, String cameraNetworkTablesName, String detectorName) {
        this.logPath = logPath;
        this.cameraNetworkTablesName = cameraNetworkTablesName;
        this.detectorName = detectorName;
    }

    protected NetworkTableEntry getLimelightNetworkTableEntry(String entryName) {
        return NetworkTableInstance.getDefault().getTable(cameraNetworkTablesName).getEntry(entryName);
    }

    @Override
    public String getName() {
        return detectorName;
    }

    @Override
    public void update() {

    }

    @Override
    public ArrayList<Optional<ObjectData>> getAllObjectData() {
        return null;
    }

    @Override
    public Optional<ObjectData> getClosestFilteredObjectData() {
        return Optional.empty();
    }

    @Override
    public void setFilter(Filter<? super ObjectData> newFilter) {
        this.filter = filter;
    }

    @Override
    public Filter<? super ObjectData> getFilter() {
        return null;
    }
}
