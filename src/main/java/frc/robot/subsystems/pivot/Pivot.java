package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.request.IRequest;
import frc.robot.hardware.signal.InputSignal;
import frc.utils.GBSubsystem;

public class Pivot extends GBSubsystem {

    private final ControllableMotor motor;
    private final InputSignal[] inputSignals;
    private final IRequest<Rotation2d> positionRequest;
    private final InputSignal<Rotation2d> positionSignal;
    private final String logPath;

    public Pivot(
            String logPath,
            String name,
            ControllableMotor motor,
            IRequest<Rotation2d> positionRequest,
            InputSignal<Rotation2d> positionSignal,
            InputSignal... inputSignals
    ) {
        super(logPath);
        this.positionSignal = positionSignal;

        this.logPath = logPath;
        this.motor = motor;
        this.inputSignals = inputSignals;
        this.positionRequest = positionRequest;
    }

    @Override
    public void subsystemPeriodic() {
        motor.updateSignals(positionSignal);
        motor.updateSignals(inputSignals);
    }

    public boolean isAtPosition(Rotation2d targetPosition, Rotation2d angleTolerance) {
        return MathUtil.isNear(targetPosition.getRotations(), positionSignal.getLatestValue().getRotations(), angleTolerance.getRotations());
    }

    public void setTargetPosition(Rotation2d targetPosition) {
        motor.applyAngleRequest(positionRequest.withSetPoint(targetPosition));
    }

    public void applyAngleRequest(IRequest<Rotation2d> request) {
        motor.applyAngleRequest(request);
    }

    public void applyDoubleRequest(IRequest<Double> request) {
        motor.applyDoubleRequest(request);
    }

    public void setBrake(boolean brake) {
        motor.setBrake(brake);
    }

    public void setPower(double power){
        motor.setPower(power);
    }
    public void stop() {
        motor.stop();
    }

}
