package frc.robot.hardware.simulations.wpilib;

import frc.robot.hardware.simulations.MechanismSimulation;
import org.wpilib.math.geometry.Rotation2d;

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
