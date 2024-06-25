package frc.utils.dashboard;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedDashboardInput;

public class LoggedTableBoolean implements LoggedDashboardInput {

    private final String key;
    private final LoggableInputs inputs;
    private final NetworkTable booleanTable;
    private boolean defaultValue;
    private boolean value;

    public LoggedTableBoolean(String table, String key) {
        this(table, key, false);
    }

    public LoggedTableBoolean(String table, String key, boolean defaultValue) {
        this.inputs = new LoggableInputs() {
            public void toLog(LogTable table) {
                table.put(LoggedTableBoolean.this.key, LoggedTableBoolean.this.value);
            }

            public void fromLog(LogTable table) {
                LoggedTableBoolean.this.value = table.get(LoggedTableBoolean.this.key, LoggedTableBoolean.this.defaultValue);
            }
        };
        this.key = key;
        this.defaultValue = defaultValue;
        this.value = defaultValue;
        this.booleanTable = NetworkTableInstance.getDefault().getTable(table);
        booleanTable.putValue(key, NetworkTableValue.makeBoolean(defaultValue));
        this.periodic();
        Logger.registerDashboardInput(this);
    }

    public void setDefault(boolean defaultValue) {
        this.defaultValue = defaultValue;
    }

    public void set(boolean value) {
        booleanTable.putValue(this.key, NetworkTableValue.makeBoolean(value));
    }

    public boolean get() {
        return this.value;
    }

    public void periodic() {
        if (!Logger.hasReplaySource()) {
            this.value = booleanTable.getValue(this.key).getBoolean();
        }
    }

}
