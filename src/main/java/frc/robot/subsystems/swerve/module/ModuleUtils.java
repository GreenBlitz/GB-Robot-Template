package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.utils.Conversions;

public class ModuleUtils {

	public enum ModulePosition {

		FRONT_LEFT(0),
		FRONT_RIGHT(1),
		BACK_LEFT(2),
		BACK_RIGHT(3);

		private final int index;

		ModulePosition(int index) {
			this.index = index;
		}

		public int getIndex() {
			return index;
		}

	}

	public static double velocityToOpenLoopVoltage(
		double velocityMetersPerSecond,
		Rotation2d steerVelocityPerSecond,
		double couplingRatio,
		Rotation2d maxVelocityPerSecond,
		double wheelDiameterMeters,
		double voltageCompensationSaturation
	) {
		Rotation2d velocityPerSecond = Conversions.distanceToAngle(velocityMetersPerSecond, wheelDiameterMeters);
		Rotation2d coupledVelocityPerSecond = getCoupledAngle(velocityPerSecond, steerVelocityPerSecond, couplingRatio);
		return velocityToVoltage(coupledVelocityPerSecond, maxVelocityPerSecond, voltageCompensationSaturation);
	}

	public static double velocityToVoltage(Rotation2d velocityPerSecond, Rotation2d maxVelocityPerSecond, double voltageCompensationSaturation) {
		double power = velocityPerSecond.getRotations() / maxVelocityPerSecond.getRotations();
		return Conversions.compensatedPowerToVoltage(power, voltageCompensationSaturation);
	}


	/**
	 * When the steer motor moves, the drive motor moves as well due to the coupling. This will affect the current position of the drive motor,
	 * so we need to remove the coupling from the velocity or the position.
	 *
	 * @param driveCoupledAngle the position or velocity
	 * @param steerAngle        the angle or velocity in angle of the module
	 * @return the distance or velocity without the coupling
	 */
	public static Rotation2d getUncoupledAngle(Rotation2d driveCoupledAngle, Rotation2d steerAngle, double couplingRatio) {
		Rotation2d steerCoupledAngle = getSteerCoupledAngle(steerAngle, couplingRatio);
		return Rotation2d.fromRotations(driveCoupledAngle.getRotations() - steerCoupledAngle.getRotations());
	}

	public static Rotation2d getCoupledAngle(Rotation2d driveUncoupledAngle, Rotation2d steerAngle, double couplingRatio) {
		Rotation2d steerCoupledAngle = getSteerCoupledAngle(steerAngle, couplingRatio);
		return Rotation2d.fromRotations(driveUncoupledAngle.getRotations() + steerCoupledAngle.getRotations());
	}

	public static Rotation2d getSteerCoupledAngle(Rotation2d steerAngle, double couplingRatio) {
		return Rotation2d.fromRotations(steerAngle.getRotations() * couplingRatio);
	}


	/**
	 * When changing direction, the module will skew since the angle motor is not at its target angle. This method will counter that by reducing
	 * the target velocity according to the angle motor's error cosine.
	 *
	 * @param targetVelocityMetersPerSecond the target velocity, in meters per second
	 * @param targetSteerAngle              the target steer angle
	 * @return the reduced target velocity in revolutions per second
	 */
	public static double reduceSkew(double targetVelocityMetersPerSecond, Rotation2d targetSteerAngle, Rotation2d currentSteerAngle) {
		double steerDeltaRadians = targetSteerAngle.getRadians() - currentSteerAngle.getRadians();
		double cosineScalar = Math.abs(Math.cos(steerDeltaRadians));
		return targetVelocityMetersPerSecond * cosineScalar;
	}

}
