package frc.utils.calibration.camera;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class CameraPoseCalibrationInputsAutoLogged extends CameraPoseCalibrationInputs implements LoggableInputs, Cloneable {
    @Override
    public void toLog(LogTable table) {
        table.put("CameraPoseFieldRelative", cameraPoseFieldRelative);
    }

    @Override
    public void fromLog(LogTable table) {
        cameraPoseFieldRelative = table.get("CameraPoseFieldRelative", cameraPoseFieldRelative);
    }

    public CameraPoseCalibrationInputsAutoLogged clone() {
        CameraPoseCalibrationInputsAutoLogged copy = new CameraPoseCalibrationInputsAutoLogged();
        copy.cameraPoseFieldRelative = this.cameraPoseFieldRelative;
        return copy;
    }
}
