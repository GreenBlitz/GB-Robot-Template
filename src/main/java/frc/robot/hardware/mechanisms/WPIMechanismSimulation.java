package frc.robot.hardware.mechanisms;

import edu.wpi.first.math.geometry.Rotation2d;

public abstract class WPIMechanismSimulation implements MechanismSimulation {

	private final double gearRatio;

	WPIMechanismSimulation(double gearRatio) {
		this.gearRatio = gearRatio;
	}

	@Override
	public Rotation2d getRotorPosition() {
		return getSystemPosition().times(gearRatio);
	}

	@Override
	public Rotation2d getRotorVelocityAnglesPerSecond() {
		return getSystemVelocityAnglesPerSecond().times(gearRatio);
	}


	@Override
	public abstract Rotation2d getSystemPosition();

	@Override
	public abstract Rotation2d getSystemVelocityAnglesPerSecond();

	@Override
	public abstract void setInputVoltage(double voltage);

	@Override
	public abstract void updateMotor();

}
