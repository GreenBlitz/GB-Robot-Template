package frc.utils.dashboard;

import edu.wpi.first.networktables.*;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardInput;

public class LoggedTableBoolean implements LoggedDashboardInput {

    private final String key;
    private final NetworkTableEntry booleanEntry;
    private final int TIME_TO_SET_BOOLEAN_MICRO_SECONDS = 1;
    private boolean value;
    private boolean defaultValue;

    public LoggedTableBoolean(String table, String key) {
        this(table, key, false);
    }

    public LoggedTableBoolean(String table, String key, boolean defaultValue) {
        this.key = key;
        this.defaultValue = defaultValue;
        this.value = this.defaultValue;
        this.booleanEntry = NetworkTableInstance.getDefault().getTable(table).getEntry(key);

        set(defaultValue);
        periodic();
        Logger.registerDashboardInput(this);
    }


    public void set(boolean value) {
        NetworkTablesJNI.setBoolean(booleanEntry.getHandle(), TIME_TO_SET_BOOLEAN_MICRO_SECONDS, value);
    }

    public boolean get() {
        return value;
    }

    public void periodic() {
        if (!Logger.hasReplaySource()) {
            value = booleanEntry.getBoolean(defaultValue);
        }
    }

}
