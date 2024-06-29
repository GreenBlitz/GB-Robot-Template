package frc.robot.simulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.utils.cycletimeutils.CycleTimeUtils;

public class FlywheelSimulation extends MotorSimulation {

    private final FlywheelSim flywheelSimulation;

    private double lastPositionRadians = 0;

    public FlywheelSimulation(FlywheelSim flywheelSimulation) {
        this.flywheelSimulation = flywheelSimulation;
    }

    public double getCurrent() {
        return flywheelSimulation.getCurrentDrawAmps();
    }

    @Override
    public Rotation2d getPosition() {
        return Rotation2d.fromRadians(lastPositionRadians);
    }

    @Override
    public Rotation2d getVelocity() {
        return Rotation2d.fromRadians(flywheelSimulation.getAngularVelocityRadPerSec());
    }

    @Override
    protected void setInputVoltage(double voltage) {
        flywheelSimulation.setInputVoltage(voltage);
    }

    @Override
    protected void updateMotor() {
        flywheelSimulation.update(CycleTimeUtils.getCurrentCycleTime());
        lastPositionRadians += getVelocity().getRadians() * CycleTimeUtils.getCurrentCycleTime();
    }

}
