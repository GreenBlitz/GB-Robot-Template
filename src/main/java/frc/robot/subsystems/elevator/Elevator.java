package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.subsystems.elevator.records.ElevatorMotorStuff;
import frc.utils.Conversions;
import org.littletonrobotics.junction.Logger;

public class Elevator extends GBSubsystem {

    private final String limitSwitchLogPath;
    private final DigitalInputInputsAutoLogged digitalInputInputsAutoLogged;
    private final ElevatorMotorStuff firstMotorStuff;
    private final ElevatorMotorStuff secondMotorStuff;
    private final ControllableMotor firstMotor;
    private final ControllableMotor secondMotor;
    private final IDigitalInput limitSwitch;
    private final ElevatorCommandsBuilder commandsBuilder;

    public Elevator(
            String logPath,
            String limitSwitchLogPath,
            ElevatorMotorStuff firstMotorStuff,
            ElevatorMotorStuff secondMotorStuff,
            IDigitalInput limitSwitch
    ) {
        super(logPath);
        this.limitSwitchLogPath = limitSwitchLogPath;
        this.firstMotorStuff = firstMotorStuff;
        this.secondMotorStuff = secondMotorStuff;
        this.firstMotor = firstMotorStuff.motor();
        this.secondMotor = firstMotorStuff.motor();
        this.limitSwitch = limitSwitch;

        this.digitalInputInputsAutoLogged = new DigitalInputInputsAutoLogged();
        this.commandsBuilder = new ElevatorCommandsBuilder(this);
    }

    public ElevatorCommandsBuilder getCommandsBuilder() {
        return commandsBuilder;
    }

    public boolean isAtBackwardsLimit() {
        return digitalInputInputsAutoLogged.debouncedValue;
    }

    public void setBrake(boolean brake) {
        firstMotor.setBrake(brake);
        secondMotor.setBrake(brake);
    }

    protected void setPower(double power) {
        firstMotor.setPower(power);
        secondMotor.setPower(power);
    }

    protected void stop() {
        firstMotor.stop();
        secondMotor.stop();
    }

    protected void setVoltage(double voltage) {
        firstMotor.applyRequest(firstMotorStuff.requests().voltageRequest().withSetPoint(voltage));
        secondMotor.applyRequest(secondMotorStuff.requests().voltageRequest().withSetPoint(voltage));
    }

    protected void setTargetPositionMeters(double targetPositionMeters) {
        Rotation2d targetPosition = convertMetersToRotations(targetPositionMeters);
        firstMotor.applyRequest(firstMotorStuff.requests().positionRequest().withSetPoint(targetPosition));
        secondMotor.applyRequest(secondMotorStuff.requests().positionRequest().withSetPoint(targetPosition));
    }

    protected void stayInPlace() {
        setTargetPositionMeters(getElevatorPositionMeters());
    }

    protected void resetMotors(Rotation2d position) {
        firstMotor.resetPosition(position);
        secondMotor.resetPosition(position);
    }

    public double getElevatorPositionMeters() {
        return convertRotationsToMeters(
                Rotation2d.fromRotations(
                        firstMotorStuff.signals().positionSignal().getLatestValue().getRotations()
                                + secondMotorStuff.signals().positionSignal().getLatestValue().getRotations() / 2
                )
        );
    }

    private void updateInputs() {
        firstMotor.updateInputs(firstMotorStuff.signals().positionSignal(), firstMotorStuff.signals().voltageSignal());
        firstMotor.updateInputs(firstMotorStuff.signals().otherSignals());
        secondMotor.updateInputs(secondMotorStuff.signals().positionSignal(), firstMotorStuff.signals().voltageSignal());
        secondMotor.updateInputs(secondMotorStuff.signals().otherSignals());
        limitSwitch.updateInputs(digitalInputInputsAutoLogged);

        log();
    }

    private void log() {
        Logger.recordOutput(getLogPath() + "PositionMeters", getElevatorPositionMeters());
        Logger.recordOutput(getLogPath() + "isAtBackwardsLimit", isAtBackwardsLimit());
        Logger.processInputs(limitSwitchLogPath, digitalInputInputsAutoLogged);
    }

    @Override
    protected void subsystemPeriodic() {
        if (getElevatorPositionMeters().getRotations() <= ElevatorConstants.MINIMUM_ACHIEVABLE_ANGLE.getRotations()) {
            resetMotors(ElevatorConstants.MINIMUM_ACHIEVABLE_ANGLE);
        }

        updateInputs();
    }

    private double convertRotationsToMeters(Rotation2d position) {
        return Conversions.angleToDistance(position, ElevatorConstants.DRUM_RADIUS);
    }

    private Rotation2d convertMetersToRotations(double meters) {
        return Conversions.distanceToAngle(meters, ElevatorConstants.DRUM_RADIUS);
    }

}