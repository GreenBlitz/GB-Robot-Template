package frc.robot.hardware.mechanisms.wpi;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.mechanisms.MechanismSimulation;

public interface WPIMechanismSimulation extends MechanismSimulation {

	@Override
	default Rotation2d getRotorPosition() {
		return getSystemPosition().times(getGearRatio());
	}

	@Override
	default Rotation2d getRotorVelocityAnglesPerSecond() {
		return getSystemVelocityAnglesPerSecond().times(getGearRatio());
	}

	double getGearRatio();

}
