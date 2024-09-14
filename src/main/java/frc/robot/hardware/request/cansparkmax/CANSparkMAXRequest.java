package frc.robot.hardware.request.cansparkmax;

import com.revrobotics.CANSparkBase;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.request.IRequest;

public class CANSparkMAXRequest implements IRequest<Double> {

    private double withSetPoint;
    private final CANSparkBase.ControlType withControlType;
    private int withPidSlot;

    private CANSparkMAXRequest(double withSetPoint, CANSparkBase.ControlType withControlType, int withPidSlot) {
        this.withSetPoint = withSetPoint;
        this.withControlType = withControlType;
        this.withPidSlot = withPidSlot;
    }

    public CANSparkMAXRequest(double positionSetPoint, int withPidSlot) {
        this(positionSetPoint, CANSparkBase.ControlType.kPosition, withPidSlot);
    }

    public CANSparkMAXRequest(Rotation2d velocitySetPoint, int withPidSlot) {
        this(velocitySetPoint.getRotations(), CANSparkBase.ControlType.kVelocity, withPidSlot);
    }

    @Override
    public CANSparkMAXRequest withSetPoint(Double setPoint) {
        withSetPoint = setPoint;
        return this;
    }

    public CANSparkMAXRequest withPidSlot(int pidSlot) {
        withPidSlot = pidSlot;
        return this;
    }

    public Double getSetPoint() {
        return withSetPoint;
    }

    public CANSparkBase.ControlType getControlType() {
        return withControlType;
    }

    public int getPidSlot() {
        return withPidSlot;
    }

}
