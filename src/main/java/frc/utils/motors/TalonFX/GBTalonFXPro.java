package frc.utils.motors.TalonFX;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class GBTalonFXPro extends TalonFX {

    public enum BusChain{
        CANIVORE("*"),
        CANBUS("");

        private final String CHAIN_NAME;

        BusChain(String chainName){
            this.CHAIN_NAME = chainName;
        }
    }

    public GBTalonFXPro(int deviceId) {
        this(deviceId, BusChain.CANBUS);
    }

    public GBTalonFXPro(int deviceId, BusChain BusChain) {
        super(deviceId, BusChain.CHAIN_NAME);
    }

    public void applyConfiguration(TalonFXConfiguration configuration) {
        super.getConfigurator().apply(configuration);
    }



    /**
     * Speeding up the gotten signals
     *
     * @param signalFrequency -> Speed of signals in hertz
     * @param statusSignals -> Signals to speed up
     */
    public void updateFrequency(double signalFrequency, StatusSignal... statusSignals){
        BaseStatusSignal.setUpdateFrequencyForAll(
                signalFrequency,
                statusSignals
        );
    }

    /**
     * Speeding up the gotten signals and delete the other signals from bus
     */
    public void optimizeBusAndSignals(double signalFrequency, StatusSignal... statusSignals){
        updateFrequency(signalFrequency, statusSignals);
        optimizeBusUtilization();
    }

    /**
     * Performs a non-blocking refresh on all provided signals.
     * IMPORTANT: Must happen before getting signals
     */
    public void refreshSignals(StatusSignal... statusSignals){
        BaseStatusSignal.refreshAll(
                statusSignals
        );
    }



    /**
     * Performs latency compensation on signal using the signalSlope and signal's
     * latency to determine the magnitude of compensation.
     */
    public double getLatencyCompensatedValue(StatusSignal<Double> value, StatusSignal<Double> valueSlope){
        refreshSignals(value, valueSlope);
        return BaseStatusSignal.getLatencyCompensatedValue(value, valueSlope);
    }

    /**
     * Performs latency compensation on position
     */
    public double getLatencyCompensatedPosition(){
        return getLatencyCompensatedValue(this.getPosition(), this.getVelocity());
    }

    /**
     * Performs latency compensation on velocity
     */
    public double getLatencyCompensatedVelocity(){
        return getLatencyCompensatedValue(this.getVelocity(), this.getAcceleration());
    }

}
