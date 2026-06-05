package frc.robot.hardware.mechanisms;

import org.wpilib.math.geometry.Rotation2d;

public interface MechanismSimulation {

	Rotation2d getRotorPosition();

	Rotation2d getRotorVelocityAnglesPerSecond();

	Rotation2d getMechanismPosition();

	Rotation2d getMechanismVelocityAnglesPerSecond();

	void setInputVoltage(double voltage);

	void updateMotor();

}
