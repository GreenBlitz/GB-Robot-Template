package frc.robot.subsystems.motorSubsystem;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.subsystems.GBSubsystem;
import frc.robot.subsystems.motorSubsystem.MotorCommandBuilder;
import frc.utils.Conversions;

public class MotorSubsystem extends GBSubsystem {

    ControllableMotor motor;
    MotorCommandBuilder commandBuilder;
    IRequest<Rotation2d> positionRequest;
    IRequest<Rotation2d> velocityRequest;
    IRequest<Double> volatgeRequest;
    InputSignal<Rotation2d> positionSignal;
    InputSignal<Rotation2d> velocitySignal;
    Rotation2d targetPosition;
    Rotation2d targetVelocity;

    public MotorSubsystem(ControllableMotor motor, String logPath) {
        super(logPath);
        this.motor = motor;
        this.commandBuilder = new MotorCommandBuilder();
    }

    public MotorSubsystem withVelocityControl(IRequest<Rotation2d> velocityRequest, InputSignal<Rotation2d> velocitySignal) {
        this.velocityRequest = velocityRequest;
        this.velocitySignal = velocitySignal;
        setTargetVelocityRotation2dPerSecond(velocitySignal.getLatestValue());
        return this;
    }

    public boolean isAtVelocity(Rotation2d targetVelocity, Rotation2d tolerance) {
        return MathUtil.isNear(targetVelocity.getRotations(), getVelocityRotation2dPerSecond().getRotations(), tolerance.getRotations());
    }

    public Rotation2d getVelocityRotation2dPerSecond() {
        if (velocitySignal == null) {
            throw new NullPointerException("the velocity request is null, try using: '.withVelocityControl'");
        }
        return velocitySignal.getLatestValue();
    }

    public void setTargetVelocityRotation2dPerSecond(Rotation2d targetVelocity) {
        if (velocityRequest == null) {
            throw new NullPointerException("the velocity request is null, try using: '.withVelocityControl'");
        }
        this.velocityRequest.withSetPoint(targetVelocity);
    }

    public void updateInputs() {
        motor.updateInputs(positionSignal, velocitySignal);
    }

    private Rotation2d convertFromMeters(double positionInMeters, double wheelDiameter) {
        return Conversions.distanceToAngle(positionInMeters, wheelDiameter);
    }

    public String getLogPath() {
        return super.getLogPath();
    }

    public void applyRequests() {
        motor.applyRequest(positionRequest);
        motor.applyRequest(velocityRequest);
        motor.applyRequest(volatgeRequest);
    }


}
