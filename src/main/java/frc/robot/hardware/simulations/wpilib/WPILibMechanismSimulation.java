package frc.robot.hardware.simulations.wpilib;

import frc.robot.hardware.simulations.MechanismSimulation;
import frc.utils.angle2d.Angle2d;

public interface WPILibMechanismSimulation extends MechanismSimulation {

  @Override
  default Angle2d getRotorPosition() {
    return getMechanismPosition().times(getGearRatio());
  }

  @Override
  default Angle2d getRotorVelocityAPS() {
    return getMechanismVelocityAPS().times(getGearRatio());
  }

  double getGearRatio();
}
