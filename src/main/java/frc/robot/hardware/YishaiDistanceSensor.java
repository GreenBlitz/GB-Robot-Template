package frc.robot.hardware;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import frc.utils.Conversions;

public class YishaiDistanceSensor {

    private static final double SCALING_SLOPE = 0.0002, SCALING_INTERCEPT_POINT = -200;

    private final DutyCycle dutyCycle;

    public YishaiDistanceSensor(DigitalInput digitalInput) {
        this.dutyCycle = new DutyCycle(digitalInput);
    }

    public double getDistanceMeters() {
        return Conversions.centimetersToMeters(dutyCycle.getHighTimeNanoseconds() * SCALING_SLOPE + SCALING_INTERCEPT_POINT);
    }

}
