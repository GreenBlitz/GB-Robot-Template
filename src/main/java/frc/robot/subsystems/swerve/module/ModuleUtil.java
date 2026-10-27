package frc.robot.subsystems.swerve.module;

import org.wpilib.math.geometry.Angle2d;
import frc.utils.Conversions;

public class ModuleUtil {

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

	public enum ControlMode {

		NONE("NONE (Target states are not being used)"),
		CUSTOM("CUSTOM (Target states are not being used)"),
		TARGET_STATE("TARGET_STATE");


		private final String toLog;

		ControlMode(String toLog) {
			this.toLog = toLog;
		}

		public String toLog() {
			return toLog;
		}

	}

	public static double velocityToOpenLoopVoltage(
		double driveVelocityMetersPerSecond,
		Angle2d steerVelocityPerSecond,
		double couplingRatio,
		Angle2d maxDriveVelocityPerSecond,
		double wheelDiameterMeters,
		double voltageCompensationSaturation
	) {
		Angle2d driveVelocityPerSecond = Conversions.distanceToAngle(driveVelocityMetersPerSecond, wheelDiameterMeters);
		Angle2d driveCoupledVelocityPerSecond = coupleDriveAngle(driveVelocityPerSecond, steerVelocityPerSecond, couplingRatio);
		return velocityToVoltage(driveCoupledVelocityPerSecond, maxDriveVelocityPerSecond, voltageCompensationSaturation);
	}

	public static double velocityToVoltage(Angle2d velocityPerSecond, Angle2d maxVelocityPerSecond, double voltageCompensationSaturation) {
		double power = velocityPerSecond.getRotations() / maxVelocityPerSecond.getRotations();
		return Conversions.compensatedPowerToVoltage(power, voltageCompensationSaturation);
	}


	/**
	 * When the steer motor moves, the drive motor moves as well due to the coupling. This will affect the current position of the drive motor,
	 * so we need to remove the coupling from the velocity or the position.
	 */
	public static Angle2d uncoupleDriveAngle(Angle2d coupledDriveAngle, Angle2d steerAngle, double couplingRatio) {
		Angle2d steerCoupledAngle = coupleSteerAngle(steerAngle, couplingRatio);
		return Angle2d.fromRotations(coupledDriveAngle.getRotations() - steerCoupledAngle.getRotations());
	}

	public static Angle2d coupleDriveAngle(Angle2d uncoupledDriveAngle, Angle2d steerAngle, double couplingRatio) {
		Angle2d steerCoupledAngle = coupleSteerAngle(steerAngle, couplingRatio);
		return Angle2d.fromRotations(uncoupledDriveAngle.getRotations() + steerCoupledAngle.getRotations());
	}

	public static Angle2d coupleSteerAngle(Angle2d steerAngle, double couplingRatio) {
		return Angle2d.fromRotations(steerAngle.getRotations() * couplingRatio);
	}

}
