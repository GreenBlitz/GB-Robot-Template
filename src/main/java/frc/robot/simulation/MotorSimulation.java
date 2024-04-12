package frc.robot.simulation;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import frc.robot.constants.RobotConstants;
import frc.robot.constants.SimulationConstants;

import java.util.ArrayList;
import java.util.List;

/**
 * A wrapper class for the WPILib default simulation classes, that'll act similarly to how the TalonFX motor controller works.
 */
public abstract class MotorSimulation {

    private static final List<MotorSimulation> REGISTERED_SIMULATIONS = new ArrayList<>();

    private final TalonFX motor;

    private final TalonFXSimState motorSimState;

    private final StatusSignal<Double> closedLoopReferenceSignal;

    protected MotorSimulation() {
        REGISTERED_SIMULATIONS.add(this);
        this.motor = new TalonFX(REGISTERED_SIMULATIONS.size() - 1);
        this.motorSimState = motor.getSimState();
        this.motorSimState.setSupplyVoltage(RobotConstants.BATTERY_VOLTAGE);
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
        motorSimState.setRawRotorPosition(getPositionRevolutions());
        motorSimState.setRotorVelocity(getVelocityRevolutionsPerSecond());
    }

    protected abstract void setInputVoltage(double voltage);

    protected abstract void updateMotor();

    public abstract double getPositionRevolutions();

    public abstract double getVelocityRevolutionsPerSecond();

    public void applyConfiguration(TalonFXConfiguration config) {
        motor.getConfigurator().apply(config);
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

    public double getProfiledSetpointRevolutions() {
        return closedLoopReferenceSignal.refresh().getValue();
    }

    public abstract double getCurrent();

}
