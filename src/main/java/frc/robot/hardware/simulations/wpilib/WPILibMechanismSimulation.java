package frc.robot.hardware.simulations.wpilib;

import org.wpilib.math.geometry.Angle2d;
import frc.robot.hardware.simulations.MechanismSimulation;

public interface WPILibMechanismSimulation extends MechanismSimulation {

	@Override
	default Angle2d getRotorPosition() {
		return getMechanismPosition().times(getGearRatio());
	}

	@Override
	default Angle2d getRotorVelocityRPS() {
		return getMechanismVelocityRPS().times(getGearRatio());
	}

	double getGearRatio();

}
