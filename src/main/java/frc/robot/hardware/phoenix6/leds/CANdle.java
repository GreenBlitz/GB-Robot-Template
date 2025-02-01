package frc.robot.hardware.phoenix6.leds;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix6.hardware.ParentDevice;
import frc.robot.hardware.ConnectedInputAutoLogged;
import frc.robot.hardware.phoenix6.Phoenix6Device;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.utils.alerts.Alert;
import frc.utils.alerts.AlertManager;
import frc.utils.alerts.PeriodicAlert;

import static edu.wpi.first.networktables.NetworkTablesJNI.isConnected;

public class CANdle {

    private final CANdleWrapper ledStrip;
    private final ConnectedInputAutoLogged connectedInput;
    private final String logPath;

    public CANdle(Phoenix6DeviceID deviceID){
        ledStrip = new CANdleWrapper(deviceID);
        this.logPath = deviceID.busChain().getChainName();
        this.connectedInput = new ConnectedInputAutoLogged();
        connectedInput.connected = true;
        //AlertManager.addAlert(new PeriodicAlert(Alert.AlertType.ERROR, logPath + "disconnectedAt", () -> !isConnected()));

    }

    public String getLogPath() {
        return logPath;
    }

    public void applyConfiguration(CANdleConfiguration config){
        if (ledStrip.applyConfiguration(config) != ErrorCode.OK){
            new Alert(Alert.AlertType.ERROR, getLogPath() + "ConfigurationFailed").report();
        }
    }

    public CANdleWrapper getDevice(){
        return ledStrip;
    }

    public ErrorCode setAnimation(Animation animation){
        return ledStrip.animate(animation);
    }

    public ErrorCode setAnimation(Animation animation, int animationSlot){
        return ledStrip.animate(animation, animationSlot);
    }

    public ErrorCode clearAnimation(int animationSlot){
        return ledStrip.clearAnimation(animationSlot);
    }

    public ErrorCode setLEDs(int red, int green, int blue, int white, int startIndex, int amountOfLEDToApply){
        return ledStrip.setLEDs(red, green, blue, white, startIndex, amountOfLEDToApply);
    }
    
    public ErrorCode setLEDs(int red, int green, int blue, int startIndex, int amountOfLEDToApply){
        return ledStrip.setLEDs(red, green, blue, 0, startIndex, amountOfLEDToApply);
    }

    public ErrorCode setLEDs(int red, int green, int blue){
        return ledStrip.setLEDs(red, green, blue);
    }
    
}
