package frc.robot.subsystems.elbow;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.request.IRequest;
import frc.utils.GBSubsystem;
import frc.utils.calibration.sysid.SysIdCalibrator;

public class Elbow extends GBSubsystem {

    private final ControllableMotor motor;
    private final IRequest<Rotation2d> positionRequest;
    private final ElbowStuff elbowStuff;
    private final ElbowCommandsBuilder commandsBuilder;

    public Elbow(String logPath, ElbowStuff elbowStuff) {
        super(logPath);
        this.motor = elbowStuff.elbow();
        this.positionRequest = elbowStuff.positionRequest();
        this.elbowStuff = elbowStuff;
        this.commandsBuilder = new ElbowCommandsBuilder(this);
    }

    @Override
    protected void subsystemPeriodic() {
        motor.updateSignals(elbowStuff.positionSignal(), elbowStuff.velocitySignal(), elbowStuff.currentSignal(), elbowStuff.voltageSignal());
    }

    public void setBrake(boolean brake) {
        motor.setBrake(brake);
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    public void stayInPlace() {
        setTargetAngle(elbowStuff.positionSignal().getLatestValue());
    }

    public void setTargetAngle(Rotation2d angle) {
        motor.applyAngleRequest(positionRequest.withSetPoint(angle));
    }

    public boolean isAtAngle(Rotation2d angle, Rotation2d tolerance) {
        return MathUtil.isNear(angle.getDegrees(), elbowStuff.positionSignal().getLatestValue().getDegrees(), tolerance.getDegrees());
    }

}
