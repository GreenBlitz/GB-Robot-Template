package frc.robot.hardware.mechanisms.wpilib;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.mechanisms.MechanismSimulation;

public interface WPILibMechanismSimulation extends MechanismSimulation {

	@Override
	default Rotation2d getRotorPosition() {
		return getMechanismPosition().times(getGearRatio());
	}

	@Override
	default Rotation2d getRotorVelocityAnglesPerSecond() {
		return getMechanismVelocityAnglesPerSecond().times(getGearRatio());
	}

	double getGearRatio();

}
