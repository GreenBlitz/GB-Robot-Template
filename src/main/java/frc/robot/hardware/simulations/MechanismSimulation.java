package frc.robot.hardware.simulations;

import org.wpilib.math.geometry.Angle2d;

public interface MechanismSimulation {

	Angle2d getRotorPosition();

	Angle2d getRotorVelocityRPS();

	Angle2d getMechanismPosition();

	Angle2d getMechanismVelocityRPS();

	void setInputVoltage(double voltage);

	void updateMotor();

}
