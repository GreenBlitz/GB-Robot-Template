package frc.robot.subsystems.funnel;

import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.motor.IMotor;
import frc.robot.hardware.signal.InputSignal;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;

public class Funnel extends GBSubsystem {

    private final IMotor motor;
    private final IDigitalInput digitalInput;
    private final DigitalInputInputsAutoLogged digitalInputsInputs;
    private final InputSignal<Double> voltageSignal;
    private final FunnelCommandBuilder commandBuilder;
    private final String logPath;

    public Funnel(FunnelStuff funnelStuff) {
        super(funnelStuff.logPath());
        this.logPath = funnelStuff.logPath();
        this.motor = funnelStuff.motor();
        this.digitalInput = funnelStuff.digitalInput();
        this.voltageSignal = funnelStuff.inputSignal();
        this.commandBuilder = new FunnelCommandBuilder(this);

        this.digitalInputsInputs = new DigitalInputInputsAutoLogged();
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    public void stop() {
        motor.stop();
    }

    public void updateInputs() {
        digitalInput.updateInputs(digitalInputsInputs);
        motor.updateSignals(voltageSignal);
    }

    public FunnelCommandBuilder getCommandBuilder() {
        return commandBuilder;
    }

    @Override
    protected void subsystemPeriodic() {
        updateInputs();
        Logger.processInputs(getLogPath() + "digitalInputs", digitalInputsInputs);
    }

}
