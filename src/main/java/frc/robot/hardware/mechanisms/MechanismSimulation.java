package frc.robot.hardware.mechanisms;

import edu.wpi.first.math.geometry.Rotation2d;

public abstract class MechanismSimulation {

	private final double gearRatio;

	MechanismSimulation(double gearRatio) {
		this.gearRatio = gearRatio;
	}

	public Rotation2d getRotorPosition() {
		return getSystemPosition().times(gearRatio);
	}

	public Rotation2d getRotorVelocityAnglesPerSecond() {
		return getSystemVelocityAnglesPerSecond().times(gearRatio);
	}


	public abstract Rotation2d getSystemPosition();

	public abstract Rotation2d getSystemVelocityAnglesPerSecond();

	public abstract void setInputVoltage(double voltage);

	public abstract void updateMotor();

}
