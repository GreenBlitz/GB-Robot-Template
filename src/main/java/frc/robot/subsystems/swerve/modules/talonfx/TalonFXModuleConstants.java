package frc.robot.subsystems.swerve.modules.talonfx;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.modules.ModuleID;
import frc.utils.Conversions;
import frc.utils.devicewrappers.TalonFXWrapper;

public class TalonFXModuleConstants {

    private final double wheelDiameterMeters;
    private final double couplingRatio;
    private final Rotation2d velocityAt12VoltsPerSecond;

    private final boolean enableFOCSteer;
    private final boolean enableFOCDrive;

    private final TalonFXWrapper steerMotor;
    private final TalonFXWrapper driveMotor;
    private final CANcoder encoder;
    private final TalonFXModuleSignals signals;


    public TalonFXModuleConstants(
            double wheelDiameterMeters,
            double couplingRatio,
            double velocityAt12VoltsMetersPerSecond,
            boolean enableFOCSteer,
            boolean enableFOCDrive,
            TalonFXConfiguration steerMotorConfig,
            TalonFXConfiguration driveMotorConfig,
            CANcoderConfiguration encoderConfig,
            ModuleID moduleID
    ) {
        this.wheelDiameterMeters = wheelDiameterMeters;
        this.couplingRatio = couplingRatio;
        this.velocityAt12VoltsPerSecond = Conversions.distanceToAngle(velocityAt12VoltsMetersPerSecond, wheelDiameterMeters);

        this.enableFOCSteer = enableFOCSteer;
        this.enableFOCDrive = enableFOCDrive;

        TalonFXModuleConfigObject moduleConfigObject = new TalonFXModuleConfigObject(steerMotorConfig, driveMotorConfig, encoderConfig, moduleID);
        this.steerMotor = moduleConfigObject.getSteerMotor();
        this.driveMotor = moduleConfigObject.getDriveMotor();
        this.encoder = moduleConfigObject.getEncoder();
        this.signals = moduleConfigObject.getSignals();
    }

    protected boolean getEnableFOCSteer(){
        return enableFOCSteer;
    }

    protected boolean getEnableFOCDrive(){
        return enableFOCDrive;
    }

    protected double getWheelDiameterMeters() {
        return wheelDiameterMeters;
    }

    protected Rotation2d getVelocityAt12VoltsPerSecond() {
        return velocityAt12VoltsPerSecond;
    }

    protected double getCouplingRatio() {
        return couplingRatio;
    }

    protected TalonFXWrapper getSteerMotor() {
        return steerMotor;
    }

    protected TalonFXWrapper getDriveMotor() {
        return driveMotor;
    }

    protected CANcoder getEncoder() {
        return encoder;
    }

    protected TalonFXModuleSignals getSignals() {
        return signals;
    }

}
