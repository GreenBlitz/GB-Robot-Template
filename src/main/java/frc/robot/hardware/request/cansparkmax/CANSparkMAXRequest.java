package frc.robot.hardware.request.cansparkmax;

import com.revrobotics.CANSparkBase;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.request.IRequest;
import java.util.function.BiFunction;

public class CANSparkMAXRequest implements IRequest<Double> {

    private double setPoint;
    private final CANSparkBase.ControlType controlType;
    private int pidSlot;
    private BiFunction<Double, Double, Double> feedforwardCalculator;

    private CANSparkMAXRequest(double setPoint, CANSparkBase.ControlType controlType, int pidSlot, BiFunction<Double, Double, Double> feedforwardCalculator) {
        this.setPoint = setPoint;
        this.controlType = controlType;
        this.pidSlot = pidSlot;
        this.feedforwardCalculator = feedforwardCalculator;
    }

    public CANSparkMAXRequest(double positionSetPoint, int pidSlot, BiFunction<Double, Double, Double> feedforwardCalculator) {
        this(positionSetPoint, CANSparkBase.ControlType.kPosition, pidSlot, feedforwardCalculator);
    }

    public CANSparkMAXRequest(Rotation2d velocitySetPoint, int pidSlot, BiFunction<Double, Double, Double> feedforwardCalculator) {
        this(velocitySetPoint.getRotations(), CANSparkBase.ControlType.kVelocity, pidSlot, feedforwardCalculator);
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

    public CANSparkMAXRequest withFeedforwardCalculator(BiFunction<Double, Double, Double> feedforwardCalculator) {
        this.feedforwardCalculator = feedforwardCalculator;
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

    public BiFunction<Double, Double, Double> getFeedforwardCalculator() {
        return feedforwardCalculator;
    }

}
