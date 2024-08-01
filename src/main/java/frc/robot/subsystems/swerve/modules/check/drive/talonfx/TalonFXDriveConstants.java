package frc.robot.subsystems.swerve.modules.check.drive.talonfx;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.utils.ctre.CTREDeviceID;
import frc.utils.devicewrappers.TalonFXWrapper;

public class TalonFXDriveConstants {

    private final TalonFXWrapper driveMotor;

    private final TalonFXDriveSignals signals;

    private final boolean enableFOC;

    public TalonFXDriveConstants(CTREDeviceID driveMotorID, TalonFXConfiguration configuration, boolean enableFOC){
        TalonFXDriveConfigObject talonFXDriveConfigObject = new TalonFXDriveConfigObject(driveMotorID, configuration);
        this.driveMotor = talonFXDriveConfigObject.getDriveMotor();
        this.signals = talonFXDriveConfigObject.getSignals();
        this.enableFOC = enableFOC;
    }

    public TalonFXWrapper getDriveMotor() {
        return driveMotor;
    }

    public TalonFXDriveSignals getSignals() {
        return signals;
    }

    public boolean getEnableFOC(){
        return enableFOC;
    }

}
