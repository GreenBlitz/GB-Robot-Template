package frc.robot.hardware;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ConnectedInputAutoLogged extends ConnectedInput implements LoggableInputs, Cloneable {
    @Override
    public void toLog(LogTable table) {
        table.put("Connected", connected);
    }

    @Override
    public void fromLog(LogTable table) {
        connected = table.get("Connected", connected);
    }

    public ConnectedInputAutoLogged clone() {
        ConnectedInputAutoLogged copy = new ConnectedInputAutoLogged();
        copy.connected = this.connected;
        return copy;
    }
}
