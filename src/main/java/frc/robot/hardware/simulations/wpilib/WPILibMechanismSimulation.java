package frc.robot.hardware.simulations.wpilib;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.simulations.MechanismSimulation;

public interface WPILibMechanismSimulation extends MechanismSimulation {

	@Override
	default Rotation2d getRotorPosition() {
		return getMechanismPosition().times(getGearRatio());
	}

	@Override
	default Rotation2d getRotorVelocityRPS() {
		return getMechanismVelocityRPS().times(getGearRatio());
	}

	double getGearRatio();

}
