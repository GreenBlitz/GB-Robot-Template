package frc.robot.subsystems.funnel;

import frc.robot.hardware.digitalinput.DigitalInputInputs;
import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.motor.IMotor;
import frc.robot.hardware.signal.InputSignal;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;

public class Funnel extends GBSubsystem {

    private final IDigitalInput beamBreaker;
    private final DigitalInputInputsAutoLogged beamBreakerInputs;
    private final IMotor motor;
    private final InputSignal[] motorSignals;
    private final String logPath;

    public Funnel(String logPath,IDigitalInput beamBreaker, IMotor motor, InputSignal... motorSignals) {
        super(logPath);
        this.logPath = logPath;
        this.beamBreaker = beamBreaker;
        this.beamBreakerInputs = new DigitalInputInputsAutoLogged();
        this.motor = motor;
        this.motorSignals = motorSignals;
    }

    @Override
    protected void subsystemPeriodic() {
        motor.updateSignals(motorSignals);
        beamBreaker.updateInputs(beamBreakerInputs);

        Logger.processInputs(logPath, beamBreakerInputs);
    }

    public void setBrake(boolean brake){
        motor.setBrake(brake);
    }

    public void setPower(double power){
        motor.setPower(power);
    }

    public void stop(){
        motor.stop();
    }
}
