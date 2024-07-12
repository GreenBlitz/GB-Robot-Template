package frc.robot.turret.simulation;

import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.simulation.SimpleMotorSimulation;
import frc.robot.turret.ITurret;
import frc.robot.turret.TurretInputsAutoLogged;

import static frc.robot.turret.simulation.SimulationConstants.GEAR_RATIO;
import static frc.robot.turret.simulation.SimulationConstants.JKG_METER_SQ;
import static frc.robot.turret.simulation.SimulationConstants.MOTOR_CONFIGURATION;


public class SimulationTurret implements ITurret {

    private final SimpleMotorSimulation motor;
    private final VelocityDutyCycle velocityDutyCycle;
    private final PositionDutyCycle positionDutyCycle;
    private final VoltageOut voltageOut;

    public SimulationTurret() {
        this.motor = new SimpleMotorSimulation(
                DCMotor.getKrakenX60(SimulationConstants.NUMBER_OF_MOTORS),
                GEAR_RATIO,
                JKG_METER_SQ
        );
        this.motor.applyConfiguration(MOTOR_CONFIGURATION);

        this.positionDutyCycle = new PositionDutyCycle(0);
        this.velocityDutyCycle = new VelocityDutyCycle(0);
        this.voltageOut = new VoltageOut(0);
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
    public void setVoltage(double voltage) {
        motor.setControl(voltageOut.withOutput(voltage));
    }

    @Override
    public void setPower(double power) {
        motor.setPower(power);
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
