package frc.robot.subsystems.swerve.modules.mk4i;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.modules.ModuleID;
import frc.utils.Conversions;
import frc.utils.devicewrappers.TalonFXWrapper;

public class MK4IModuleConstants {

    private final double wheelDiameterMeters;
    private final double couplingRatio;
    private final Rotation2d maxVelocityPerSecond;

    private final boolean enableFOCSteer;
    private final boolean enableFOCDrive;

    private final TalonFXWrapper steerMotor;
    private final TalonFXWrapper driveMotor;
    private final CANcoder encoder;
    private final MK4IModuleSignals signals;


    public MK4IModuleConstants(
            double wheelDiameterMeters,
            double couplingRatio,
            double maxVelocityMetersPerSecond,
            boolean enableFOCSteer,
            boolean enableFOCDrive,
            TalonFXConfiguration steerMotorConfig,
            TalonFXConfiguration driveMotorConfig,
            CANcoderConfiguration encoderConfig,
            ModuleID moduleID
    ) {
        this.wheelDiameterMeters = wheelDiameterMeters;
        this.couplingRatio = couplingRatio;
        this.maxVelocityPerSecond = Conversions.distanceToAngle(maxVelocityMetersPerSecond, wheelDiameterMeters);

        this.enableFOCSteer = enableFOCSteer;
        this.enableFOCDrive = enableFOCDrive;

        MK4IModuleConfigObject moduleConfigObject = new MK4IModuleConfigObject(steerMotorConfig, driveMotorConfig, encoderConfig, moduleID);
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

    protected Rotation2d getMaxVelocityPerSecond() {
        return maxVelocityPerSecond;
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

    protected MK4IModuleSignals getSignals() {
        return signals;
    }

}
