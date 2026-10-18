package frc.robot.vision.cameras.limelight.inputs;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class NeuralDetectionInputsAutoLogged extends NeuralDetectionInputs implements LoggableInputs, Cloneable {
    @Override
    public void toLog(LogTable table) {
        table.put("Target2dValues", target2dValues);
        table.put("RawDetections", rawDetections);
    }

    @Override
    public void fromLog(LogTable table) {
        target2dValues = table.get("Target2dValues", target2dValues);
        rawDetections = table.get("RawDetections", rawDetections);
    }

    public NeuralDetectionInputsAutoLogged clone() {
        NeuralDetectionInputsAutoLogged copy = new NeuralDetectionInputsAutoLogged();
        copy.target2dValues = this.target2dValues;
        copy.rawDetections = this.rawDetections.clone();
        return copy;
    }
}
