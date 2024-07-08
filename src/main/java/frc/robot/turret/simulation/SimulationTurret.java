package frc.robot.turret.simulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.turret.ITurret;
import frc.robot.turret.TurretInputsAutoLogged;

import static frc.robot.turret.simulation.SimulationConstants.POSITION_PID_CONTROLLER;
import static frc.robot.turret.simulation.SimulationConstants.VELOCITY_PID_CONTROLLER;

public class SimulationTurret implements ITurret {

    private DCMotorSim motor;
    private TurretInputsAutoLogged lastInputs;

    public SimulationTurret() {
        motor = new DCMotorSim(
                DCMotor.getKrakenX60(SimulationConstants.AMOUNT_OF_MOTORS),
                SimulationConstants.GEARING,
                SimulationConstants.JKG_METER_SQ
        );
        lastInputs = new TurretInputsAutoLogged();
    }

    @Override
    public void setVelocity(double velocity) {
        VELOCITY_PID_CONTROLLER.setSetpoint(velocity);
        motor.setInputVoltage(VELOCITY_PID_CONTROLLER.calculate(lastInputs.velocity));
    }

    @Override
    public void setPosition(Rotation2d angle) {
        POSITION_PID_CONTROLLER.setSetpoint(angle.getRadians());
        motor.setInputVoltage(POSITION_PID_CONTROLLER.calculate(lastInputs.position.getRadians()));
    }

    @Override
    public void stop() {
        motor.setInputVoltage(0);
    }

    @Override
    public void updateInputs(TurretInputsAutoLogged inputs) {
        inputs.position = Rotation2d.fromRadians(motor.getAngularPositionRad());
        inputs.velocity = motor.getAngularVelocityRadPerSec();
    }

}
