package frc.robot.hardware.mechanisms;

import edu.wpi.first.math.geometry.Rotation2d;

public interface MechanismSimulation {


	Rotation2d getRotorPosition();

	Rotation2d getRotorVelocityAnglesPerSecond();


	Rotation2d getSystemPosition();

	Rotation2d getSystemVelocityAnglesPerSecond();

	void setInputVoltage(double voltage);

	void updateMotor();

}
