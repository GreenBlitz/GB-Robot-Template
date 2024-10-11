package frc.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.constants.MathConstants;

public class Conversions {

	public static final double MAG_TICKS = 4096;
	public static final double HUNDRED_MILLISECONDS_PER_SECONDS = 10;
	public static final double SECONDS_PER_MINUTE = 60;


	/**
	 * Converts ticks from a Mag Encoder to angle.
	 *
	 * @param systemGearRatio the gear ratio of the system, when motor/ration = system
	 * @param magTicks        ticks from a Mag Encoder
	 * @return angle
	 */
	public static Rotation2d magTicksToAngle(double magTicks, double systemGearRatio) {
		return Rotation2d.fromRotations(magTicks / MAG_TICKS / systemGearRatio);
	}

	/**
	 * Converts angle to Mag Encoder ticks.
	 *
	 * @param systemGearRatio the gear ratio of the system, when motor/ration = system
	 * @param angle           angle
	 * @return Mag Encoder ticks
	 */
	public static double angleToMagTicks(Rotation2d angle, double systemGearRatio) {
		return angle.getRotations() * MAG_TICKS * systemGearRatio;
	}

	/**
	 * Converts motor data to system data. This can be velocity, position, acceleration, etc.
	 *
	 * @param motorData the motor data
	 * @param gearRatio the gear ratio between the motor and the system. 2 means that 2 motor rotations are 1 system rotation.
	 * @return the system data
	 */
	public static double motorToSystem(double motorData, double gearRatio) {
		return motorData / gearRatio;
	}

	/**
	 * Converts system data to motor data. This can be velocity, position, acceleration, etc.
	 *
	 * @param systemData the system data
	 * @param gearRatio  the gear ratio between the motor and the system. 2 means that 2 motor rotations are 1 system rotation.
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
	 * Converts a frequency from per 100 milliseconds to per minute.
	 *
	 * @param frequency the frequency per 100 milliseconds
	 * @return the frequency per minute
	 */
	public static double per100MillisecondsToPerMinute(double frequency) {
		return perSecondToPerMinute(per100MillisecondsToPerSecond(frequency));
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
	 * Converts a frequency from per 100 milliseconds to per second.
	 *
	 * @param frequency the frequency per 100 milliseconds
	 * @return the frequency per second
	 */
	public static double per100MillisecondsToPerSecond(double frequency) {
		return frequency * HUNDRED_MILLISECONDS_PER_SECONDS;
	}

	/**
	 * Converts a frequency from per minute to per 100 milliseconds.
	 *
	 * @param frequency the frequency per minute
	 * @return the frequency per 100 milliseconds
	 */
	public static double perMinuteToPer100Milliseconds(double frequency) {
		return perSecondToPer100Milliseconds(frequency) / SECONDS_PER_MINUTE;
	}

	/**
	 * Converts a frequency from per second to per 100 milliseconds.
	 *
	 * @param frequency the frequency per second
	 * @return the frequency per 100 milliseconds
	 */
	public static double perSecondToPer100Milliseconds(double frequency) {
		return frequency / HUNDRED_MILLISECONDS_PER_SECONDS;
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
	 * Converts a time from milliseconds to seconds
	 *
	 * @param milliSeconds the milliseconds
	 * @return the seconds
	 */
	public static double milliSecondsToSeconds(double milliSeconds) {
		return milliSeconds / 1e3;
	}

	/**
	 * Converts angle to distance.
	 *
	 * @param angle         the angle
	 * @param wheelDiameter the wheel diameter
	 * @return the distance
	 */
	public static double angleToDistance(Rotation2d angle, double wheelDiameter) {
		return angle.getRotations() * wheelDiameter * MathConstants.HALF_CIRCLE.getRadians();
	}

	/**
	 * Converts distance to angle.
	 *
	 * @param distance      the distance
	 * @param wheelDiameter the wheel diameter
	 * @return the angle
	 */
	public static Rotation2d distanceToAngle(double distance, double wheelDiameter) {
		return Rotation2d.fromRotations(distance / (wheelDiameter * MathConstants.HALF_CIRCLE.getRadians()));
	}

	/**
	 * Converts a target output voltage to a percentage output when voltage compensation is enabled. The voltage compensation saturation
	 * determines what voltage represents 100% output. The compensated power is the voltage represented by a percentage of the saturation
	 * voltage.
	 *
	 * @param voltage    the target voltage output
	 * @param saturation the configured saturation which represents 100% output
	 * @return the percentage output to achieve the target voltage
	 */
	public static double voltageToCompensatedPower(double voltage, double saturation) {
		return voltage / saturation;
	}

	/**
	 * Converts a target output percentage output to voltage when voltage compensation is enabled. The voltage compensation saturation determines
	 * what voltage represents 100% output. The compensated power is the voltage represented by a percentage of the saturation voltage.
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
