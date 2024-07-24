package frc.utils.devicewrappers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.utils.ctre.CTREDeviceID;

public class TalonFXWrapper extends TalonFX {

    public TalonFXWrapper(int deviceId) {
        this(new CTREDeviceID(deviceId));
    }

    public TalonFXWrapper(CTREDeviceID ctreDeviceID) {
        this(ctreDeviceID, new TalonFXConfiguration());
    }

    public TalonFXWrapper(CTREDeviceID ctreDeviceID, TalonFXConfiguration configuration) {
        super(ctreDeviceID.ID(), ctreDeviceID.busChain().getChainName());
        applyConfiguration(configuration);
    }

    public StatusCode applyConfiguration(TalonFXConfiguration configuration) {
        return super.getConfigurator().apply(configuration);
    }


    /**
     * Performs latency compensation on signal using the signalSlope and signal's
     * latency to determine the magnitude of compensation.
     */
    private double getLatencyCompensatedValue(StatusSignal<Double> value, StatusSignal<Double> valueSlope) {
        return BaseStatusSignal.getLatencyCompensatedValue(value, valueSlope);
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
