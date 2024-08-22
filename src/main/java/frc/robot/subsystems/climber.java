package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.ControlState;
import frc.robot.hardware.IMotor;
import frc.robot.hardware.MotorInputsAutoLogged;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;

public class climber extends GBSubsystem {
    int CLIMBING_SLOT = 1;
    IMotor climber;
    MotorInputsAutoLogged motorInputs;
    MetersInputsAutoLogged metersInputs;

    public climber(){
        super("climber");
        motorInputs = new MotorInputsAutoLogged();
    }

    double angleToMeters(Rotation2d rotation2d){
        return 1;
    }

    public void setTargetAngle(Rotation2d angle) {
        climber.setTargetAngle(angle, ControlState.MOTION_MAGIC, 0);
    }
    public void setClimbingAngleTargetAngle(Rotation2d angle) {
        climber.setTargetAngle(angle, ControlState.MOTION_MAGIC, CLIMBING_SLOT);
    }

    @Override
    protected void subsystemPeriodic() {
        climber.updateInputs(motorInputs);
        Logger.processInputs("arm", motorInputs);
    }

    public boolean isAtAngle(){
        return false;
    }

}
