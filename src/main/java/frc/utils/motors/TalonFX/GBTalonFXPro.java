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
        super(deviceId);
    }

    public GBTalonFXPro(int deviceId, BusChain BusChain) {
        super(deviceId, BusChain.CHAIN_NAME);
    }

    public void applyConfiguration(TalonFXConfiguration configuration) {
        super.getConfigurator().apply(configuration);
    }

    public void optimizeCanBusUtilization(double signalFrequency, StatusSignal... statusSignals){
        BaseStatusSignal.setUpdateFrequencyForAll(
                signalFrequency,
                statusSignals
        );
        this.optimizeBusUtilization();
    }

    public double getLatencyCompensatedValue(StatusSignal<Double> value, StatusSignal<Double> valueSlope){
        return BaseStatusSignal.getLatencyCompensatedValue(value, valueSlope);
    }

    public double getLatencyCompensatedPosition(){
        return BaseStatusSignal.getLatencyCompensatedValue(this.getPosition(), this.getVelocity());
    }

    public double getLatencyCompensatedVelocity(){
        return BaseStatusSignal.getLatencyCompensatedValue(this.getVelocity(), this.getAcceleration());
    }

}
