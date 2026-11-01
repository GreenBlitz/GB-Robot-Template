package frc.robot.hardware.digitalinput;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class DigitalInputInputsAutoLogged extends DigitalInputInputs implements LoggableInputs, Cloneable {
    @Override
    public void toLog(LogTable table) {
        table.put("DebouncedValue", debouncedValue);
        table.put("NonDebouncedValue", nonDebouncedValue);
    }

    @Override
    public void fromLog(LogTable table) {
        debouncedValue = table.get("DebouncedValue", debouncedValue);
        nonDebouncedValue = table.get("NonDebouncedValue", nonDebouncedValue);
    }

    public DigitalInputInputsAutoLogged clone() {
        DigitalInputInputsAutoLogged copy = new DigitalInputInputsAutoLogged();
        copy.debouncedValue = this.debouncedValue;
        copy.nonDebouncedValue = this.nonDebouncedValue;
        return copy;
    }
}
