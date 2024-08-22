package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.ControlState;
import frc.robot.hardware.IMotor;
import frc.robot.hardware.MotorInputsAutoLogged;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;

public class elevator extends GBSubsystem {

    IMotor elevator;
    MotorInputsAutoLogged motorInputs;
    MetersInputsAutoLogged metersInputs;

    public elevator(){
        super("arm");
        motorInputs = new MotorInputsAutoLogged();
    }

    double angleToMeters(Rotation2d rotation2d){
        return 1;
    }

    public void setTargetAngle(Rotation2d angle) {
        elevator.setTargetAngle(angle, ControlState.MOTION_MAGIC);
    }

    @Override
    protected void subsystemPeriodic() {
        elevator.updateInputs(motorInputs);
        Logger.processInputs("arm", motorInputs);
    }

    public boolean isAtAngle(){
        return false;
    }

}
