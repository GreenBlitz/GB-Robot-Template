package frc.robot.simulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.utils.cycletimeutils.CycleTimeUtils;

/**
 * This class simulates a motor. Used for Subsystems like rollers and intake.
 */
public class SimpleMotorSimulation extends MotorSimulation {

    private final DCMotorSim motorSimulation;

    public SimpleMotorSimulation(DCMotorSim motorSimulation) {
        this.motorSimulation = motorSimulation;
    }

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
        motorSimulation.update(CycleTimeUtils.getCurrentCycleTime());
    }

}
