package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.hardware.interfaces.VelocityPositionRequest;
import frc.robot.subsystems.GBSubsystem;

public class LinearElevator extends GBSubsystem {

    protected final ControllableMotor motor;
    private final InputSignal<Double> voltageSignal;
    private final InputSignal<Double> currentSignal;
    private final InputSignal<Rotation2d> positionSignal;
    private final InputSignal<Rotation2d> velocitySignal;
    private final VelocityPositionRequest request;
    private final double elevatorHeightCM;
    private Rotation2d targetPosition;

    public LinearElevator(
            String logPath,
            ControllableMotor motor,
            InputSignal<Double> voltageSignal,
            InputSignal<Double> currentSignal,
            InputSignal<Rotation2d> positionSignal,
            InputSignal<Rotation2d> velocitySignal,
            VelocityPositionRequest velocityPositionRequest
    ) {
        super(logPath);
        this.motor = motor;
        this.voltageSignal = voltageSignal;
        this.currentSignal = currentSignal;
        this.positionSignal = positionSignal;
        this.velocitySignal = velocitySignal;
        this.request = velocityPositionRequest;
        //this.commandsBuilder = new RollerCommandsBuilder(this);
        this.motor.resetPosition(Rotation2d.fromRotations(0));
        this.targetPosition = Rotation2d.fromRotations(0);
        //setDefaultCommand(commandsBuilder.stop());
    }

    public void updateTargetPosition(Rotation2d targetPosition) {
        this.targetPosition = targetPosition;
    }

    public void stop() {
        motor.stop();
    }

    public void setBrake(boolean brake) {
        motor.setBrake(brake);
    }

    public Double getVoltage() {
        return voltageSignal.getLatestValue();
    }

    public Double getCurrent() {
        return currentSignal.getLatestValue();
    }

    public Rotation2d getPosition() {
        return positionSignal.getLatestValue();
    }

    public Rotation2d getVelocity() {
        return velocitySignal.getLatestValue();
    }

}
