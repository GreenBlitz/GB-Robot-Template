package frc.robot.subsystems.swerve.module.extrainputs;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ModuleInputsAutoLogged extends ModuleInputs implements LoggableInputs, Cloneable {
    @Override
    public void toLog(LogTable table) {
        table.put("IsClosedLoop", isClosedLoop);
        table.put("TargetState", targetVelocity);
        table.put("ControlMode", controlMode);
    }

    @Override
    public void fromLog(LogTable table) {
        isClosedLoop = table.get("IsClosedLoop", isClosedLoop);
        targetVelocity = table.get("TargetState", targetVelocity);
        controlMode = table.get("ControlMode", controlMode);
    }

    public ModuleInputsAutoLogged clone() {
        ModuleInputsAutoLogged copy = new ModuleInputsAutoLogged();
        copy.isClosedLoop = this.isClosedLoop;
        copy.targetVelocity= this.targetVelocity;
        copy.controlMode = this.controlMode;
        return copy;
    }
}
