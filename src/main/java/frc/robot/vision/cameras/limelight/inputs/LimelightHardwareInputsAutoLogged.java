package frc.robot.vision.cameras.limelight.inputs;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class LimelightHardwareInputsAutoLogged extends LimelightHardwareInputs implements LoggableInputs, Cloneable {
    @Override
    public void toLog(LogTable table) {
        table.put("TemperatureCelsius", temperatureCelsius);
        table.put("Connected", connected);
    }

    @Override
    public void fromLog(LogTable table) {
        temperatureCelsius = table.get("TemperatureCelsius", temperatureCelsius);
        connected = table.get("Connected", connected);
    }

    public LimelightHardwareInputsAutoLogged clone() {
        LimelightHardwareInputsAutoLogged copy = new LimelightHardwareInputsAutoLogged();
        copy.temperatureCelsius = this.temperatureCelsius;
        copy.connected = this.connected;
        return copy;
    }
}
