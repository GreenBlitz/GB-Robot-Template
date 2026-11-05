package frc.robot.hardware.simulations;

import frc.utils.costomUnits.Angle2d;

public interface MechanismSimulation {

	Angle2d getRotorPosition();

	Angle2d getRotorVelocityAPS();

	Angle2d getMechanismPosition();

	Angle2d getMechanismVelocityAPS();

	void setInputVoltage(double voltage);

	void updateMotor();

}
