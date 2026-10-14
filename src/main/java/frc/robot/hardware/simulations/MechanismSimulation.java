package frc.robot.hardware.simulations;

import edu.wpi.first.math.geometry.Rotation2d;

public interface MechanismSimulation {

	Rotation2d getRotorPosition();

	Rotation2d getRotorVelocityRPS();

	Rotation2d getMechanismPosition();

	Rotation2d getMechanismVelocityRPS();

	void setInputVoltage(double voltage);

	void updateMotor();

}
