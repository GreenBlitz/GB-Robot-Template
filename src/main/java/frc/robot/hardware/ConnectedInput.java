package frc.robot.hardware;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

@AutoLog
public class ConnectedInput implements LoggableInputs {

	public boolean connected = false;

    @Override
    public void toLog(LogTable table) {

    }

    @Override
    public void fromLog(LogTable table) {

    }
}
