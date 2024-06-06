package frc.robot.simulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.utils.cycletimeutils.CycleTimeUtils;

public class SimpleMotorSimulation extends MotorSimulation {

    private final DCMotorSim motorSimulation;

    public SimpleMotorSimulation(DCMotor gearbox, double gearRatio, double momentOfInertia) {
        this.motorSimulation = new DCMotorSim(gearbox, gearRatio, momentOfInertia);
    }

    @Override
    public double getCurrent() {
        return motorSimulation.getCurrentDrawAmps();
    }

    @Override
    public Rotation2d getPosition() {
        return Rotation2d.fromRadians(motorSimulation.getAngularPositionRad());
    }

    @Override
    public Rotation2d getVelocity() {
        return Rotation2d.fromRadians(motorSimulation.getAngularVelocityRadPerSec());
    }

    @Override
    protected void setInputVoltage(double voltage) {
        motorSimulation.setInputVoltage(voltage);
    }

    @Override
    protected void updateMotor() {
        motorSimulation.update(CycleTimeUtils.getDefaultCycleTime());
    }

}
