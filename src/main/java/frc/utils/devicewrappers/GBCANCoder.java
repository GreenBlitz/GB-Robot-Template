package frc.utils.devicewrappers;

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import frc.robot.constants.Phoenix6Constants;

public class GBCANCoder extends CANcoder {

    public GBCANCoder(int deviceId) {
        this(deviceId, Phoenix6Constants.CANBUS_NAME);
    }

    public GBCANCoder(int deviceId, String busChain) {
        super(deviceId, busChain);
    }

    public void applyConfiguration(CANcoderConfiguration configuration) {
        super.getConfigurator().apply(configuration);
    }

    /**
     * Speeding up the gotten signals and delete the other signals from bus
     */
    public void optimizeBusAndSignals(double signalFrequency, StatusSignal... statusSignals) {
        updateFrequency(signalFrequency, statusSignals);
        optimizeBusUtilization();
    }

    /**
     * Speeding up the gotten signals
     *
     * @param signalFrequency -> Speed of signals in hertz
     * @param statusSignals   -> Signals to speed up
     */
    public void updateFrequency(double signalFrequency, StatusSignal... statusSignals) {
        BaseStatusSignal.setUpdateFrequencyForAll(signalFrequency, statusSignals);
    }

    /**
     * Performs latency compensation on position
     */
    public double getLatencyCompensatedPosition() {
        return getLatencyCompensatedValue(this.getPosition(), this.getVelocity());
    }

    /**
     * Performs latency compensation on signal using the signalSlope and signal's
     * latency to determine the magnitude of compensation.
     */
    public double getLatencyCompensatedValue(StatusSignal<Double> value, StatusSignal<Double> valueSlope) {
        refreshSignals(value, valueSlope);
        return BaseStatusSignal.getLatencyCompensatedValue(value, valueSlope);
    }

    /**
     * Performs a non-blocking refresh on all provided signals.
     * IMPORTANT: Must happen before getting signals
     */
    public void refreshSignals(StatusSignal... statusSignals) {
        BaseStatusSignal.refreshAll(statusSignals);
    }
}
