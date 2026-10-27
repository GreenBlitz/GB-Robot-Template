package frc.utils;

import frc.utils.costomUnits.Angle2d;
import org.wpilib.networktables.DoubleEntry;
import org.wpilib.networktables.NetworkTableInstance;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedNetworkInput;

public class LoggedNetworkAngle2d extends LoggedNetworkInput {

    private final String key;
    private final DoubleEntry entry;
    private Angle2d defaultValue = Angle2d.fromDegrees(0);
    private Angle2d value;

    public LoggedNetworkAngle2d(String key) {
        this.key = key;
        this.entry = NetworkTableInstance.getDefault().getDoubleTopic(key + "InDegrees").getEntry(0.0);
        this.value = defaultValue;
        Logger.registerDashboardInput(this);
    }

    public LoggedNetworkAngle2d(String key, Angle2d defaultValue) {
        this(key);
        setDefault(defaultValue);
        this.value = defaultValue;
    }

    public void setDefault(Angle2d defaultValue) {
        this.defaultValue = defaultValue;
        entry.set(entry.get(defaultValue.getDegrees()));
    }

    public void set(Angle2d value) {
        entry.set(value.getDegrees());
    }

    public Angle2d get() {
        return value;
    }

    private final LoggableInputs inputs = new LoggableInputs() {

        public void toLog(LogTable table) {
            table.put(removeSlash(key), value);
        }

        public void fromLog(LogTable table) {
            value = table.get(removeSlash(key + "InDegrees"), defaultValue);
        }

    };

    public void periodic() {
        if (!Logger.hasReplaySource()) {
            value = Angle2d.fromDegrees(entry.get());
        }
        Logger.processInputs(prefix, inputs);
    }

}
