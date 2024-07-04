package frc.utils;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.constants.MathConstants;

public class Conversions {

    public static final double MAG_TICKS = 4096;
    public static final double DEGREES_PER_REVOLUTIONS = 360;
    public static final double HUNDRED_MILLIESECONDS_PER_SECONDS = 10;
    public static final double SECONDS_PER_MINUTE = 60;

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
     * Converts revolutions to degrees.
     *
     * @param revolutions revolutions
     * @return degrees
     */
    public static double revolutionsToDegrees(double revolutions) {
        return revolutions * DEGREES_PER_REVOLUTIONS;
    }

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
     * Converts degrees to Mag Encoder ticks.
     *
     * @param degrees degrees
     * @return Mag Encoder ticks
     */
    public static double degreesToMagTicks(double degrees) {
        return revolutionsToMagTicks(degreesToRevolutions(degrees));
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
     * Converts motor data to system data.
     * This can be velocity, position, acceleration, etc.
     *
     * @param motorData the motor data
     * @param gearRatio the gear ratio between the motor and the system. 2 means that 2 motor rotations are 1 system
     * rotation.
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
     * @param gearRatio the gear ratio between the motor and the system. 2 means that 2 motor rotations are 1 system
     * rotation.
     * @return the motor data
     */
    public static double systemToMotor(double systemData, double gearRatio) {
        return systemData * gearRatio;
    }

    /**
     * Converts a frequency from per minute to per second.
     *
     * @param frequency the frequency per minute
     * @return the frequency per second
     */
    public static double perMinuteToPerSecond(double frequency) {
        return frequency / SECONDS_PER_MINUTE;
    }

    /**
     * Converts a frequency from per 100millieseconds to per minute.
     *
     * @param frequency the frequency per 100millieseconds
     * @return the frequency per minute
     */
    public static double per100MilliesecondsToPerMinute(double frequency) {
        return perSecondToPerMinute(per100MilliesecondsToPerSecond(frequency));
    }

    /**
     * Converts a frequency from per second to per minute.
     *
     * @param frequency the frequency per second
     * @return the frequency per minute
     */
    public static double perSecondToPerMinute(double frequency) {
        return frequency * SECONDS_PER_MINUTE;
    }

    /**
     * Converts a frequency from per 100 millieseconds to per second.
     *
     * @param frequency the frequency per 100 millieseconds
     * @return the frequency per second
     */
    public static double per100MilliesecondsToPerSecond(double frequency) {
        return frequency * HUNDRED_MILLIESECONDS_PER_SECONDS;
    }

    /**
     * Converts a frequency from per minute to per 100 millieseconds.
     *
     * @param frequency the frequency per minute
     * @return the frequency per 100millieseconds
     */
    public static double perMinuteToPer100Millieseconds(double frequency) {
        return perSecondToPer100Millieseconds(frequency) / SECONDS_PER_MINUTE;
    }

    /**
     * Converts a frequency from per second to per 100millieseconds.
     *
     * @param frequency the frequency per second
     * @return the frequency per 100millieseconds
     */
    public static double perSecondToPer100Millieseconds(double frequency) {
        return frequency / HUNDRED_MILLIESECONDS_PER_SECONDS;
    }


    /**
     * Converts a micro seconds to seconds
     *
     * @param microSeconds the micro seconds
     * @return the seconds
     */
    public static double microSecondsToSeconds(double microSeconds) {
        return microSeconds / 1e6;
    }

    /**
     * Converts revolutions to distance.
     *
     * @param revolutions the revolutions
     * @param wheelDiameter the wheel diameter
     * @return the distance
     */
    public static double revolutionsToDistance(double revolutions, double wheelDiameter) {
        return revolutions * wheelDiameter * MathConstants.HALF_CIRCLE.getRadians();
    }

    /**
     * Converts distance to revolutions.
     *
     * @param distance the distance
     * @param wheelDiameter the wheel diameter
     * @return the revolutions
     */
    public static double distanceToRevolutions(double distance, double wheelDiameter) {
        return distance / (wheelDiameter * MathConstants.HALF_CIRCLE.getRadians());
    }

    /**
     * Converts a target output voltage to a percentage output when voltage compensation is enabled.
     * The voltage compensation saturation determines what voltage represents 100% output.
     * The compensated power is the voltage represented by a percentage of the saturation voltage.
     *
     * @param voltage the target voltage output
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
     * @param power the target percentage output
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
     * @param percentage the percentage of speed
     * @return the scaled constraints
     */
    public static TrapezoidProfile.Constraints scaleConstraints(TrapezoidProfile.Constraints constraints, double percentage) {
        return new TrapezoidProfile.Constraints(
                constraints.maxVelocity * (percentage / 100),
                constraints.maxAcceleration * (percentage / 100)
        );
    }

}
