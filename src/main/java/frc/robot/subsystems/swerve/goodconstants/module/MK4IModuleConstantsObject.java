package frc.robot.subsystems.swerve.goodconstants.module;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.modules.mk4i.MK4IModuleConfigObject;
import frc.utils.Conversions;
import frc.utils.ctre.CTREDeviceID;

public class MK4IModuleConstantsObject {

    private final double wheelDiameter;
    private final double couplingRatio;
    private final Rotation2d maxVelocityPerSecond;

    private final boolean enableFocSteer;
    private final boolean enableFocDrive;

    private final MK4IModuleConfigObject moduleConfigObject;


    public MK4IModuleConstantsObject(
            double wheelDiameter,
            double couplingRatio,
            double maxVelocityMetersPerSecond,
            boolean enableFocSteer,
            boolean enableFocDrive,
            boolean steerInverted,
            boolean driveInverted,
            TalonFXConfiguration steerMotorConfig,
            TalonFXConfiguration driveMotorConfig,
            CANcoderConfiguration encoderConfig,
            CTREDeviceID steerMotorDeviceID,
            CTREDeviceID driveMotorDeviceID,
            CTREDeviceID encoderDeviceID
    ) {
        this.wheelDiameter = wheelDiameter;
        this.couplingRatio = couplingRatio;
        this.maxVelocityPerSecond = Conversions.distanceToAngle(maxVelocityMetersPerSecond, wheelDiameter);

        this.enableFocSteer = enableFocSteer;
        this.enableFocDrive = enableFocDrive;

        this.moduleConfigObject = new MK4IModuleConfigObject(
                steerMotorDeviceID, steerInverted, steerMotorConfig,
                driveMotorDeviceID, driveInverted, driveMotorConfig,
                encoderDeviceID, encoderConfig
        );
    }



}
