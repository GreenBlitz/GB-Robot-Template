package frc.robot.hardware;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class FollowerInputsAutoLogged extends FollowerInputs implements LoggableInputs, Cloneable {
    @Override
    public void toLog(LogTable table) {
        table.put("FollowerData", followerData);
    }

    @Override
    public void fromLog(LogTable table) {
        followerData = table.get("FollowerData", followerData);
    }

    public FollowerInputsAutoLogged clone() {
        FollowerInputsAutoLogged copy = new FollowerInputsAutoLogged();
        copy.followerData = this.followerData;
        return copy;
    }
}
