package frc.utils;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class Conversions {
    public static final double
            MAG_TICKS = 4096,
            DEGREES_PER_REVOLUTIONS = 360,
            HUNDRED_MS_PER_SEC = 10,
            SEC_PER_MIN = 60;

    /**
     * Converts ticks from a Mag Encoder to revolutions.
     *
     * @param magTicks ticks from a Mag Encoder
     * @return revolutions
     */
    public static double magTicksToRevolutions(double magTicks) {
        return magTicks / MAG_TICKS;
    }

    /**
     * Converts revolutions to Mag Encoder ticks.
     *
     * @param revolutions revolutions
     * @return Mag Encoder ticks
     */
    public static double revolutionsToMagTicks(double revolutions) {
        return revolutions * MAG_TICKS;
    }

    /**
     * Converts degrees to revolutions.
     *
     * @param degrees degrees
     * @return revolutions
     */
    public static double degreesToRevolutions(double degrees) {
        return degrees / DEGREES_PER_REVOLUTIONS;
    }

    /**
     * Converts revolutions to degrees.
     *
     * @param revolutions revolutions
     * @return degrees
     */
    public static double revolutionsToDegrees(double revolutions) {
        return revolutions * DEGREES_PER_REVOLUTIONS;
    }

    /**
     * Converts ticks from a Mag Encoder to degrees.
     *
     * @param magTicks ticks from a Mag Encoder
     * @return degrees
     */
    public static double magTicksToDegrees(double magTicks) {
        return revolutionsToDegrees(magTicksToRevolutions(magTicks));
    }

    /**
     * Converts degrees to Mag Encoder ticks.
     *
     * @param degrees degrees
     * @return Mag Encoder ticks
     */
    public static double degreesToMagTicks(double degrees) {
        return revolutionsToMagTicks(degreesToRevolutions(degrees));
    }

    /**
     * Converts motor data to system data.
     * This can be velocity, position, acceleration, etc.
     *
     * @param motorData the motor data
     * @param gearRatio the gear ratio between the motor and the system. 2 means that 2 motor rotations are 1 system
     *                  rotation.
     * @return the system data
     */
    public static double motorToSystem(double motorData, double gearRatio) {
        return motorData / gearRatio;
    }

    /**
     * Converts system data to motor data.
     * This can be velocity, position, acceleration, etc.
     *
     * @param systemData the system data
     * @param gearRatio  the gear ratio between the motor and the system. 2 means that 2 motor rotations are 1 system
     *                   rotation.
     * @return the motor data
     */
    public static double systemToMotor(double systemData, double gearRatio) {
        return systemData * gearRatio;
    }

    /**
     * Addition.
     *
     * @param x the first number.
     * @param y the second number.
     * @return the added value.
     */
    public static double plus(double x, double y) {
        return x + y;
    }

    /**
     * Applies an offset to the position read from the motor in order to compensate for a sensor offset.
     *
     * @param position the target position of the motor.
     * @param offset   the encoder value when the system is on zero position.
     * @return the actual position of the motor offset.
     */
    public static double offsetRead(double position, double offset) {
        return position - offset;
    }

    /**
     * Converts a frequency from per 100ms to per second.
     *
     * @param frequency the frequency per 100ms
     * @return the frequency per second
     */
    public static double perHundredMsToPerSecond(double frequency) {
        return frequency * HUNDRED_MS_PER_SEC;
    }

    /**
     * Converts a frequency from per second to per 100ms.
     *
     * @param frequency the frequency per second
     * @return the frequency per 100ms
     */
    public static double perSecondToPerHundredMs(double frequency) {
        return frequency / HUNDRED_MS_PER_SEC;
    }

    /**
     * Converts a frequency from per second to per minute.
     *
     * @param frequency the frequency per second
     * @return the frequency per minute
     */
    public static double perSecondToPerMinute(double frequency) {
        return frequency * SEC_PER_MIN;
    }

    /**
     * Converts a frequency from per minute to per second.
     *
     * @param frequency the frequency per minute
     * @return the frequency per second
     */
    public static double perMinuteToPerSecond(double frequency) {
        return frequency / SEC_PER_MIN;
    }

    /**
     * Converts a frequency from per 100ms to per minute.
     *
     * @param frequency the frequency per 100ms
     * @return the frequency per minute
     */
    public static double perHundredMsToPerMinute(double frequency) {
        return perSecondToPerMinute(perHundredMsToPerSecond(frequency));
    }

    /**
     * Converts a frequency from per minute to per 100ms.
     *
     * @param frequency the frequency per minute
     * @return the frequency per 100ms
     */
    public static double perMinToPerSec(double frequency) {
        return perSecondToPerHundredMs(frequency) / SEC_PER_MIN;
    }

    /**
     * Converts revolutions to distance.
     *
     * @param revolutions   the revolutions
     * @param wheelDiameter the wheel diameter
     * @return the distance
     */
    public static double revolutionsToDistance(double revolutions, double wheelDiameter) {
        return revolutions * wheelDiameter * Math.PI;
    }

    /**
     * Converts distance to revolutions.
     *
     * @param distance      the distance
     * @param wheelDiameter the wheel diameter
     * @return the revolutions
     */
    public static double distanceToRevolutions(double distance, double wheelDiameter) {
        return distance / (wheelDiameter * Math.PI);
    }

    /**
     * Converts a target output voltage to a percentage output when voltage compensation is enabled.
     * The voltage compensation saturation determines what voltage represents 100% output.
     * The compensated power is the voltage represented by a percentage of the saturation voltage.
     *
     * @param voltage    the target voltage output
     * @param saturation the configured saturation which represents 100% output
     * @return the percentage output to achieve the target voltage
     */
    public static double voltageToCompensatedPower(double voltage, double saturation) {
        return voltage / saturation;
    }

    /**
     * Converts a target output percentage output to voltage when voltage compensation is enabled.
     * The voltage compensation saturation determines what voltage represents 100% output.
     * The compensated power is the voltage represented by a percentage of the saturation voltage.
     *
     * @param power      the target percentage output
     * @param saturation the configured saturation which represents 100% output
     * @return the percentage output to achieve the target voltage
     */
    public static double compensatedPowerToVoltage(double power, double saturation) {
        return power * saturation;
    }

    /**
     * Scales a TrapezoidProfile.Constraints object by a given percentage.
     *
     * @param constraints the constraints to scale
     * @param percentage  the percentage of speed
     * @return the scaled constraints
     */
    public static TrapezoidProfile.Constraints scaleConstraints(TrapezoidProfile.Constraints constraints, double percentage) {
        return new TrapezoidProfile.Constraints(constraints.maxVelocity * (percentage / 100), constraints.maxAcceleration * (percentage / 100));
    }
}