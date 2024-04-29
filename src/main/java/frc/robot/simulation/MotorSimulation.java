package frc.robot.simulation;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.SimulationConstants;
import frc.utils.devicewrappers.GBTalonFXPro;

import java.util.ArrayList;
import java.util.List;

/**
 * A wrapper class for the WPILib default simulation classes, that'll act similarly to how the TalonFX motor controller works.
 */
public abstract class MotorSimulation {

    private static final List<MotorSimulation> REGISTERED_SIMULATIONS = new ArrayList<>();

    private final GBTalonFXPro motor;

    private final TalonFXSimState motorSimState;

    private final StatusSignal<Double> closedLoopReferenceSignal;

    protected MotorSimulation() {
        REGISTERED_SIMULATIONS.add(this);
        this.motor = new GBTalonFXPro(REGISTERED_SIMULATIONS.size());
        this.motorSimState = motor.getSimState();
        this.motorSimState.setSupplyVoltage(SimulationConstants.BATTERY_VOLTAGE);
        this.closedLoopReferenceSignal = motor.getClosedLoopReference();
        this.closedLoopReferenceSignal.setUpdateFrequency(1.0 / SimulationConstants.TIME_STEP);
    }

    public static void updateRegisteredSimulations() {
        for (MotorSimulation motorSimulation : REGISTERED_SIMULATIONS) {
            motorSimulation.updateSimulation();
        }
    }

    private void updateSimulation() {
        setInputVoltage(motorSimState.getMotorVoltage());
        updateMotor();

        motorSimState.setRawRotorPosition(getPosition().getRotations());
        motorSimState.setRotorVelocity(getVelocity().getRotations());
    }

    public void applyConfiguration(TalonFXConfiguration config) {
        motor.applyConfiguration(config);
    }

    public void setPower(double power) {
        motor.set(power);
    }

    public void stop() {
        motor.stopMotor();
    }

    public void setControl(ControlRequest request) {
        motor.setControl(request);
    }

    public double getVoltage() {
        return motor.getMotorVoltage().getValue();
    }

    public Rotation2d getProfiledSetPoint() {
        return Rotation2d.fromRotations(closedLoopReferenceSignal.refresh().getValue());
    }

    public abstract double getCurrent();

    protected abstract void setInputVoltage(double voltage);

    protected abstract void updateMotor();

    public abstract Rotation2d getPosition();

    public abstract Rotation2d getVelocity();

}

