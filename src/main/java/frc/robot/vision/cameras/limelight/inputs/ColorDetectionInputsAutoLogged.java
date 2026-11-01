package frc.robot.vision.cameras.limelight.inputs;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ColorDetectionInputsAutoLogged extends ColorDetectionInputs implements LoggableInputs, Cloneable {
    @Override
    public void toLog(LogTable table) {
        table.put("Target2dValues", target2dValues);
        table.put("RawTargets", rawTargets);
    }

    @Override
    public void fromLog(LogTable table) {
        target2dValues = table.get("Target2dValues", target2dValues);
        rawTargets = table.get("RawTargets", rawTargets);
    }

    public ColorDetectionInputsAutoLogged clone() {
        ColorDetectionInputsAutoLogged copy = new ColorDetectionInputsAutoLogged();
        copy.target2dValues = this.target2dValues;
        copy.rawTargets = this.rawTargets.clone();
        return copy;
    }
}
