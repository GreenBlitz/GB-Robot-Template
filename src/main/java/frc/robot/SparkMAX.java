package frc.robot;

import com.ctre.phoenix6.StatusSignal;
import frc.robot.hardware.IDevice;
import frc.robot.hardware.signal.InputSignal;
import frc.robot.hardware.signal.cansparkmax.ISparkMaxSignal;
import frc.robot.hardware.signal.cansparkmax.SparkMaxAngleSignal;
import frc.utils.devicewrappers.SparkMaxWrapper;
import org.littletonrobotics.junction.Logger;

import java.util.LinkedList;

public class SparkMAX implements IDevice {

    private SparkMaxWrapper motor;
    private final String logPath;

    public SparkMAX (String logPath,SparkMaxWrapper motor){
        this.motor = motor;
        this.logPath = logPath;
    }

    @Override
    public boolean isConnected() {
        return motor.getBusVoltage() > 0;
    }

    @Override
    public void updateSignals(InputSignal... signals) {
        for (InputSignal signal : signals){
            if(signal instanceof ISparkMaxSignal){
                Logger.processInputs(logPath, signal);
            }
        }
    }
}
