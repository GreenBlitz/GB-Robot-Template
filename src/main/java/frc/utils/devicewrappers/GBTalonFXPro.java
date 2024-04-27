package frc.utils.devicewrappers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class GBTalonFXPro extends TalonFX {

    public GBTalonFXPro(int deviceId, String BusChain) {
        this(deviceId, BusChain, new TalonFXConfiguration());
    }

    public GBTalonFXPro(int deviceId, String BusChain, TalonFXConfiguration configuration) {
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