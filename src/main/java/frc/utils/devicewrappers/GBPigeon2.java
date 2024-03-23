package frc.utils.devicewrappers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.constants.Phoenix6Constants;

public class GBPigeon2 extends Pigeon2 {

    public GBPigeon2(int deviceId) {
        this(deviceId, Phoenix6Constants.CANBUS_NAME);
    }

    public GBPigeon2(int deviceId, String busChain) {
        super(deviceId, busChain);
    }

    public void applyConfiguration(Pigeon2Configuration configuration) {
        super.getConfigurator().apply(configuration);
    }


    /**
     * Speeding up the gotten signals
     *
     * @param signalFrequency -> Speed of signals in hertz
     * @param statusSignals   -> Signals to speed up
     */
    public void updateFrequency(double signalFrequency, StatusSignal... statusSignals) {
        BaseStatusSignal.setUpdateFrequencyForAll(
                signalFrequency,
                statusSignals
        );
    }

    /**
     * Speeding up the gotten signals and delete the other signals from bus
     */
    public void optimizeBusAndSignals(double signalFrequency, StatusSignal... statusSignals) {
        updateFrequency(signalFrequency, statusSignals);
        optimizeBusUtilization();
    }

    /**
     * Performs a non-blocking refresh on all provided signals.
     * IMPORTANT: Must happen before getting signals
     */
    public void refreshSignals(StatusSignal... statusSignals) {
        BaseStatusSignal.refreshAll(
                statusSignals
        );
    }
}
