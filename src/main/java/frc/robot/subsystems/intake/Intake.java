package frc.robot.subsystems.intake;

import frc.robot.hardware.digitalinput.DigitalInputInputs;
import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.motor.IMotor;
import frc.robot.hardware.signal.InputSignal;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;

public class Intake extends GBSubsystem {

    private final IMotor motor;
    private final IDigitalInput digitalInput;
    private final DigitalInputInputsAutoLogged digitalInputsInputs;
    private final InputSignal<Double> voltage;

    public Intake(String logPath, IMotor motor, IDigitalInput digitalInput, InputSignal<Double> voltage) {
        super(logPath);

        this.motor = motor;
        this.digitalInput = digitalInput;
        this.voltage = voltage;

        digitalInputsInputs = new DigitalInputInputsAutoLogged();
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    public void stop() {
        motor.stop();
    }

    public void updateInputs() {
        digitalInput.updateInputs(digitalInputsInputs);
    }

    @Override
    protected void subsystemPeriodic() {
        updateInputs();
//        Logger.processInputs(getLogPath() + "digital inputs", digitalInputsInputs);
    }
}
