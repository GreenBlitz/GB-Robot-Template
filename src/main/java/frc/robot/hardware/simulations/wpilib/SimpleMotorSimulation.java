package frc.robot.hardware.simulations.wpilib;

import frc.utils.angle2d.Angle2d;
import frc.utils.time.TimeUtil;
import org.wpilib.simulation.DCMotorSim;

public class SimpleMotorSimulation implements WPILibMechanismSimulation {

  private final DCMotorSim motorSimulation;

  public SimpleMotorSimulation(DCMotorSim motorSimulation) {
    this.motorSimulation = motorSimulation;
  }

  @Override
  public Angle2d getMechanismPosition() {
    return Angle2d.fromRadians(motorSimulation.getAngularPosition());
  }

  @Override
  public Angle2d getMechanismVelocityAPS() {
    return Angle2d.fromRadians(motorSimulation.getAngularVelocity());
  }

  @Override
  public void setInputVoltage(double voltage) {
    motorSimulation.setInputVoltage(voltage);
  }

  @Override
  public void updateMotor() {
    motorSimulation.update(TimeUtil.getLatestCycleTimeSeconds());
  }

  @Override
  public double getGearRatio() {
    return motorSimulation.getGearing();
  }
}
