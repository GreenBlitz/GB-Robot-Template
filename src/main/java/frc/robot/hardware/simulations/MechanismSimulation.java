package frc.robot.hardware.simulations;

import frc.utils.costomUnits.Angle2d;

public interface MechanismSimulation {

	Angle2d getRotorPosition();

	Angle2d getRotorVelocityRPS();

	Angle2d getMechanismPosition();

	Angle2d getMechanismVelocityRPS();

	void setInputVoltage(double voltage);

	void updateMotor();

}
