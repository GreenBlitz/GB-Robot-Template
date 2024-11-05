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
		Rotation2d coupledVelocityPerSecond = coupleDriveAngle(velocityPerSecond, steerVelocityPerSecond, couplingRatio);
		return velocityToVoltage(coupledVelocityPerSecond, maxVelocityPerSecond, voltageCompensationSaturation);
	}

	public static double velocityToVoltage(Rotation2d velocityPerSecond, Rotation2d maxVelocityPerSecond, double voltageCompensationSaturation) {
		double power = velocityPerSecond.getRotations() / maxVelocityPerSecond.getRotations();
		return Conversions.compensatedPowerToVoltage(power, voltageCompensationSaturation);
	}


	/**
	 * When the steer motor moves, the drive motor moves as well due to the coupling. This will affect the current position of the drive motor,
	 * so we need to remove the coupling from the velocity or the position.
	 */
	public static Rotation2d uncoupleDriveAngle(Rotation2d coupledDriveAngle, Rotation2d steerAngle, double couplingRatio) {
		Rotation2d steerCoupledAngle = coupleSteerAngle(steerAngle, couplingRatio);
		return Rotation2d.fromRotations(coupledDriveAngle.getRotations() - steerCoupledAngle.getRotations());
	}

	public static Rotation2d coupleDriveAngle(Rotation2d uncoupledDriveAngle, Rotation2d steerAngle, double couplingRatio) {
		Rotation2d steerCoupledAngle = coupleSteerAngle(steerAngle, couplingRatio);
		return Rotation2d.fromRotations(uncoupledDriveAngle.getRotations() + steerCoupledAngle.getRotations());
	}

	public static Rotation2d coupleSteerAngle(Rotation2d steerAngle, double couplingRatio) {
		return Rotation2d.fromRotations(steerAngle.getRotations() * couplingRatio);
	}


	/**
	 * When changing direction, the module will skew since the angle motor is not at its target angle. This method will counter that by reducing
	 * the target velocity according to the angle motor's error cosine.
	 */
	public static double reduceSkew(double targetDriveVelocityMetersPerSecond, Rotation2d targetSteerPosition, Rotation2d currentSteerPosition) {
		double steerDeltaRadians = targetSteerPosition.getRadians() - currentSteerPosition.getRadians();
		double cosineScalar = Math.abs(Math.cos(steerDeltaRadians));
		return targetDriveVelocityMetersPerSecond * cosineScalar;
	}

}
