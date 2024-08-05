package frc.utils.dashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardInput;

public class LoggedTableBoolean implements LoggedDashboardInput {

    private final String key;
    private final NetworkTable booleanTable;
    private boolean value;

    public LoggedTableBoolean(String table, String key) {
        this(table, key, false);
    }

    public LoggedTableBoolean(String table, String key, boolean defaultValue) {
        this.key = key;
        this.value = defaultValue;
        this.booleanTable = NetworkTableInstance.getDefault().getTable(table);

        set(defaultValue);
        periodic();
        Logger.registerDashboardInput(this);
    }


    public void set(boolean value) {
        booleanTable.putValue(key, NetworkTableValue.makeBoolean(value));
    }

    public boolean get() {
        return value;
    }

    public void periodic() {
        if (!Logger.hasReplaySource()) {
            value = booleanTable.getValue(key).getBoolean();
        }
    }

}