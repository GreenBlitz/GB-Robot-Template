package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.ControlState;
import frc.robot.hardware.IMotor;
import frc.robot.hardware.MotorInputsAutoLogged;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;

public class arm extends GBSubsystem {

    IMotor arm;
    MotorInputsAutoLogged inputs;

    public arm(){
        super("arm");
        inputs = new MotorInputsAutoLogged();
    }

    public void setTargetAngle(Rotation2d angle) {
        arm.setTargetAngle(angle, ControlState.MOTION_MAGIC);
    }

    @Override
    protected void subsystemPeriodic() {
        arm.updateInputs(inputs);
        Logger.processInputs("arm", inputs);
    }

    public boolean isAtAngle(){
        return false;
    }

}
