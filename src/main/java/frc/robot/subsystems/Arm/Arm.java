package frc.robot.subsystems.Arm;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.subsystems.GBSubsystem;
import frc.utils.ToleranceUtils;

public class Arm extends GBSubsystem {
    public final Rotation2d MAXIMUM_POSITION = Rotation2d.fromDegrees(130);
    public final Rotation2d MINIMUM_POSITION = Rotation2d.fromDegrees(-130);
    public final Rotation2d STARTING_POSITION = Rotation2d.fromDegrees(0);

    private final ControllableMotor motor;
    private final ArmCommandBuilder commandBuilder;
    private final IRequest<Rotation2d> positionRequest;
    private final IRequest<Double> voltageRequest;
    private final InputSignal<Rotation2d> positionSignal;
    private final InputSignal<Double> voltageSignal;

    public Arm(String logPath, ControllableMotor motor, IRequest<Rotation2d> positionRequest, IRequest<Double> voltageRequest, InputSignal<Rotation2d> positionSignal, InputSignal<Double> voltageSignal) {
        super(logPath);
        this.motor = motor;
        this.commandBuilder = new ArmCommandBuilder(this);
        this.positionRequest = positionRequest;
        this.voltageRequest = voltageRequest;
        this.positionSignal = positionSignal;
        this.voltageSignal = voltageSignal;

        motor.resetPosition(STARTING_POSITION);
        updateInputs();
    }

    public ControllableMotor getMotor() {
        return motor;
    }

    public ArmCommandBuilder getCommandBuilder() {
        return commandBuilder;
    }

    public IRequest<Rotation2d> getPositionRequest() {
        return positionRequest;
    }

    public IRequest<Double> getVoltageRequest() {
        return voltageRequest;
    }

    public InputSignal<Rotation2d> getPosition() {
        return positionSignal;
    }

    public InputSignal<Double> getVoltage() {
        return voltageSignal;
    }

    private void updateInputs() {
        motor.updateInputs(positionSignal, voltageSignal);
    }

    public void setBrake(boolean brake){
        motor.setBrake(brake);
    }

    protected void setVoltage(double voltage) {
        motor.applyRequest(voltageRequest.withSetPoint(voltage));
    }

    protected void setTargetPosition(Rotation2d angle) {
        motor.applyRequest(positionRequest.withSetPoint(angle));
    }

    protected void stayInPlace(){
        setTargetPosition(positionSignal.getLatestValue());
    }

    public boolean isAtPosition(Rotation2d angle, Rotation2d tolerance) {
        return ToleranceUtils.isNearWrapped(angle, positionSignal.getLatestValue(), tolerance);
    }

}
