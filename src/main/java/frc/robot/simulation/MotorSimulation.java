package frc.robot.simulation;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.utils.batteryutils.BatteryUtils;
import frc.utils.cycletimeutils.CycleTimeUtils;
import frc.utils.devicewrappers.TalonFXWrapper;


/**
 * A wrapper class for the WPILib default simulation classes, that'll act similarly to how the TalonFX motor controller works.
 */
abstract class MotorSimulation {

    private final TalonFXWrapper motor;

    private final TalonFXSimState motorSimulationState;

    private final StatusSignal<Double> closedLoopReferenceSignal;

    protected MotorSimulation() {
        SimulationManager.addSimulation(this);
        this.motor = SimulationManager.createNewMotorForSimulation();
        this.motorSimulationState = motor.getSimState();
        this.motorSimulationState.setSupplyVoltage(BatteryUtils.DEFAULT_VOLTAGE);
        this.closedLoopReferenceSignal = motor.getClosedLoopReference();
        this.closedLoopReferenceSignal.setUpdateFrequency(1.0 / CycleTimeUtils.DEFAULT_CYCLE_TIME_SECONDS);
    }

    protected void updateSimulation() {
        setInputVoltage(motorSimulationState.getMotorVoltage());
        updateMotor();
        motorSimulationState.setRawRotorPosition(getPosition().getRotations());
        motorSimulationState.setRotorVelocity(getVelocity().getRotations());
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

    protected abstract void setInputVoltage(double voltage);

    protected abstract void updateMotor();

    public abstract Rotation2d getPosition();

    public abstract Rotation2d getVelocity();

}
