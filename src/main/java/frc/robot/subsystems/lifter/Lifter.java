package frc.robot.subsystems.lifter;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.digitalinput.DigitalInputInputs;
import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.motor.ControllableMotor;
import frc.utils.Conversions;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;

public class Lifter extends GBSubsystem {

    private final ControllableMotor motor;
    private final IDigitalInput limitSwitch;
    private final LifterStuff lifterStuff;
    private final LifterCommandsBuilder lifterCommandsBuilder;
    private final DigitalInputInputsAutoLogged digitalInputInputs;

    public Lifter(LifterStuff lifterStuff) {
        super(lifterStuff.logPath());
        this.motor = lifterStuff.motor();
        this.limitSwitch = lifterStuff.limitSwitch();
        this.lifterStuff = lifterStuff;
        this.digitalInputInputs = new DigitalInputInputsAutoLogged();
        this.lifterCommandsBuilder = new LifterCommandsBuilder(this);
        motor.resetPosition(new Rotation2d());

        updateInputs();
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    public void stop() {
        motor.stop();
    }

    public void setBrake(boolean brake) {
        motor.setBrake(brake);
    }

    public boolean isHigher(double expectedPositionMeters) {
        return expectedPositionMeters < convertToMeters(lifterStuff.positionSignal().getLatestValue());
    }

    public boolean isLower(double expectedPositionMeters) {
        return !isHigher(expectedPositionMeters);
    }

    public boolean isLimitSwitchPressed(){
        return digitalInputInputs.debouncedValue;
    }

    public LifterStuff getLifterStuff() {
        return lifterStuff;
    }

    public LifterCommandsBuilder getCommandsBuilder() {
        return lifterCommandsBuilder;
    }

    @Override
    protected void subsystemPeriodic() {
        updateInputs();
    }

    private void updateInputs() {
        motor.updateSignals(lifterStuff.positionSignal());
        motor.updateSignals(lifterStuff.otherSignals());

        limitSwitch.updateInputs(digitalInputInputs);
        Logger.processInputs(getLogPath() + "isLifterDown", digitalInputInputs);
        Logger.recordOutput("lifter position", convertToMeters(lifterStuff.positionSignal().getLatestValue()));

    }

    private double convertToMeters(Rotation2d motorPosition) {
        return Conversions.angleToDistance(motorPosition, lifterStuff.drumRadius());
    }
}

