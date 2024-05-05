package frc.utils.devicewrappers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.Phoenix6Constants;

public class TalonFXWrapper extends TalonFX {

    public TalonFXWrapper(int deviceId) {
        this(deviceId, Phoenix6Constants.CANBUS_NAME, new TalonFXConfiguration());
    }

    public TalonFXWrapper(int deviceId, TalonFXConfiguration configuration) {
        this(deviceId, Phoenix6Constants.CANBUS_NAME, configuration);
    }

    public TalonFXWrapper(int deviceId, String BusChain) {
        this(deviceId, BusChain, new TalonFXConfiguration());
    }

    public TalonFXWrapper(int deviceId, String BusChain, TalonFXConfiguration configuration) {
        super(deviceId, BusChain);
        applyConfiguration(configuration);
    }

    public void applyConfiguration(TalonFXConfiguration configuration) {
        super.getConfigurator().apply(configuration);
    }

    /**
     * Performs latency compensation on signal using the signalSlope and signal's
     * latency to determine the magnitude of compensation.
     */
    public double getLatencyCompensatedValue(StatusSignal<Double> value, StatusSignal<Double> valueSlope) {
        return BaseStatusSignal.getLatencyCompensatedValue(value.refresh(), valueSlope.refresh());
    }

    /**
     * Performs latency compensation on velocity
     */
    public double getLatencyCompensatedVelocity() {
        return getLatencyCompensatedValue(this.getVelocity(), this.getAcceleration());
    }

    /**
     * Performs latency compensation on position
     */
    public double getLatencyCompensatedPosition() {
        return getLatencyCompensatedValue(this.getPosition(), this.getVelocity());
    }

}
