package frc.robot.hardware.request.cansparkmax;

import com.revrobotics.CANSparkBase;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.request.IRequest;

public class CANSparkMAXRequest implements IRequest<Double> {

    private double setPoint;
    private final CANSparkBase.ControlType controlType;
    private int pidSlot;

    public CANSparkMAXRequest(double setPoint, CANSparkBase.ControlType controlType, int pidSlot) {
        this.setPoint = setPoint;
        this.controlType = controlType;
        this.pidSlot = pidSlot;
    }

    public CANSparkMAXRequest(double positionSetPoint, int pidSlot) {
        this(positionSetPoint, CANSparkBase.ControlType.kPosition, pidSlot);
    }

    public CANSparkMAXRequest(Rotation2d velocitySetPoint, int pidSlot) {
        this(velocitySetPoint.getRotations(), CANSparkBase.ControlType.kVelocity, pidSlot);
    }

    @Override
    public CANSparkMAXRequest withSetPoint(Double setPoint) {
        this.setPoint = setPoint;
        return this;
    }

    public CANSparkMAXRequest withPidSlot(int pidSlot) {
        this.pidSlot = pidSlot;
        return this;
    }

    public double getSetPoint() {
        return setPoint;
    }

    public CANSparkBase.ControlType getControlType() {
        return controlType;
    }

    public int getPidSlot() {
        return pidSlot;
    }

}
