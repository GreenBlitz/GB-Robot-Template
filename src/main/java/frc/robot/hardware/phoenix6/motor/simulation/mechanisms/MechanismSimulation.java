package frc.robot.hardware.phoenix6.motor.simulation.mechanisms;

import edu.wpi.first.math.geometry.Rotation2d;

public abstract class MechanismSimulation {

	private final double gearRatio;

	MechanismSimulation(double gearRatio) {
		this.gearRatio = gearRatio;
	}

	public Rotation2d getRotorPositionRotations() {
		return getSystemPositionRotations().times(gearRatio);
	}

	public Rotation2d getRotorVelocityRotationsPerSecond() {
		return getSystemVelocityRotationsPerSecond().times(gearRatio);
	}

	public abstract Rotation2d getSystemPositionRotations();

	public abstract Rotation2d getSystemVelocityRotationsPerSecond();

	public abstract void setInputVoltage(double voltage);

	public abstract void updateMotor();

}
