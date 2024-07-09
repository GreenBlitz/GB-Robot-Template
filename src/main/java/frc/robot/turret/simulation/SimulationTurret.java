package frc.robot.turret.simulation;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.simulation.SimpleMotorSimulation;
import frc.robot.turret.ITurret;
import frc.robot.turret.TurretInputsAutoLogged;
import frc.utils.cycletimeutils.CycleTimeUtils;

import static frc.robot.turret.simulation.SimulationConstants.GEARING;
import static frc.robot.turret.simulation.SimulationConstants.JKG_METER_SQ;


public class SimulationTurret implements ITurret {

    private SimpleMotorSimulation motor;
    private VelocityDutyCycle velocityDutyCycle;
    private PositionDutyCycle positionDutyCycle;

    public SimulationTurret() {
        motor = new SimpleMotorSimulation(
                DCMotor.getKrakenX60(SimulationConstants.AMOUNT_OF_MOTORS),
                GEARING,
                JKG_METER_SQ
        );

        positionDutyCycle = new PositionDutyCycle(0);
        velocityDutyCycle = new VelocityDutyCycle(0);
    }

    @Override
    public void setVelocity(double velocity) {
        motor.setControl(velocityDutyCycle.withVelocity(velocity));
    }

    @Override
    public void setPosition(Rotation2d angle) {
        motor.setControl(positionDutyCycle.withPosition(angle.getRotations()));
    }

    @Override
    public void stop() {
        motor.stop();
    }

    @Override
    public void updateInputs(TurretInputsAutoLogged inputs) {
        inputs.position = motor.getPosition();
        inputs.velocity = motor.getVelocity();

        inputs.targetPosition = Rotation2d.fromRotations(velocityDutyCycle.Velocity);
        inputs.targetVelocity = Rotation2d.fromRotations(positionDutyCycle.Position);
    }

}
