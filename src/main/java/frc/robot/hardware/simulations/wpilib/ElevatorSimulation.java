package frc.robot.hardware.simulations.wpilib;

import frc.utils.Conversions;
import frc.utils.angle2d.Angle2d;
import frc.utils.time.TimeUtil;
import org.wpilib.simulation.ElevatorSim;

public class ElevatorSimulation implements WPILibMechanismSimulation {

  private final ElevatorSim elevatorSimulation;
  private final double drumDiameter;
  private final double gearRatio;

  public ElevatorSimulation(ElevatorSim elevatorSimulation, double drumDiameter, double gearRatio) {
    this.elevatorSimulation = elevatorSimulation;
    this.drumDiameter = drumDiameter;
    this.gearRatio = gearRatio;
  }

  @Override
  public Angle2d getMechanismPosition() {
    return Conversions.distanceToAngle(elevatorSimulation.getPosition(), drumDiameter);
  }

  @Override
  public Angle2d getMechanismVelocityAPS() {
    return Conversions.distanceToAngle(elevatorSimulation.getVelocity(), drumDiameter);
  }

  @Override
  public void setInputVoltage(double voltage) {
    elevatorSimulation.setInputVoltage(voltage);
  }

  @Override
  public void updateMotor() {
    elevatorSimulation.update(TimeUtil.getLatestCycleTimeSeconds());
  }

  @Override
  public double getGearRatio() {
    return gearRatio;
  }
}
