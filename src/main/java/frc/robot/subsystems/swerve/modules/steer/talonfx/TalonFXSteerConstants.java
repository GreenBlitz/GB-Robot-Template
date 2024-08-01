package frc.robot.subsystems.swerve.modules.steer.talonfx;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.utils.ctre.CTREDeviceID;
import frc.utils.devicewrappers.TalonFXWrapper;

public class TalonFXSteerConstants {

    protected static final int NO_ENCODER_ID = -1;

    private final TalonFXWrapper steerMotor;
    private final TalonFXSteerSignals signals;
    private final boolean enableFOC;

    public TalonFXSteerConstants(
            CTREDeviceID steerMotorID,
            TalonFXConfiguration configuration,
            boolean enableFOC
    ){
        this(steerMotorID, NO_ENCODER_ID, configuration, enableFOC);
    }

    public TalonFXSteerConstants(
            CTREDeviceID steerMotorID,
            int encoderID,
            TalonFXConfiguration configuration,
            boolean enableFOC
    ){
        TalonFXSteerConfigObject talonFXSteerConfigObject = new TalonFXSteerConfigObject(steerMotorID, encoderID, configuration);
        this.steerMotor = talonFXSteerConfigObject.getSteerMotor();
        this.signals = talonFXSteerConfigObject.getSignals();
        this.enableFOC = enableFOC;
    }


    public TalonFXWrapper getSteerMotor() {
        return steerMotor;
    }

    public TalonFXSteerSignals getSignals() {
        return signals;
    }

    public boolean getEnableFOC(){
        return enableFOC;
    }

}
