package frc.robot.subsystems.swerve.modules.mk4i;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.modules.ModuleID;
import frc.utils.Conversions;
import frc.utils.devicewrappers.TalonFXWrapper;

public class MK4IModuleConstants {

    private final double wheelDiameter;
    private final double couplingRatio;
    private final Rotation2d maxVelocityPerSecond;

    private final boolean enableFocSteer;
    private final boolean enableFocDrive;

    private final TalonFXWrapper steerMotor;
    private final TalonFXWrapper driveMotor;
    private final CANcoder encoder;
    private final MK4IModuleSignals signals;


    public MK4IModuleConstants(
            double wheelDiameter,
            double couplingRatio,
            double maxVelocityMetersPerSecond,
            boolean enableFocSteer,
            boolean enableFocDrive,
            TalonFXConfiguration steerMotorConfig,
            TalonFXConfiguration driveMotorConfig,
            CANcoderConfiguration encoderConfig,
            ModuleID moduleID
    ) {
        this.wheelDiameter = wheelDiameter;
        this.couplingRatio = couplingRatio;
        this.maxVelocityPerSecond = Conversions.distanceToAngle(maxVelocityMetersPerSecond, wheelDiameter);

        this.enableFocSteer = enableFocSteer;
        this.enableFocDrive = enableFocDrive;

        MK4IModuleConfigObject moduleConfigObject = new MK4IModuleConfigObject(steerMotorConfig, driveMotorConfig, encoderConfig, moduleID);
        this.steerMotor = moduleConfigObject.steerMotor();
        this.driveMotor = moduleConfigObject.driveMotor();
        this.encoder = moduleConfigObject.encoder();
        this.signals = moduleConfigObject.signals();
    }

    public boolean enableFocSteer(){
        return enableFocSteer;
    }

    public boolean enableFocDrive(){
        return enableFocDrive;
    }

    public double wheelDiameter() {
        return wheelDiameter;
    }

    public Rotation2d maxVelocityPerSecond() {
        return maxVelocityPerSecond;
    }

    public double couplingRatio() {
        return couplingRatio;
    }

    public TalonFXWrapper steerMotor() {
        return steerMotor;
    }

    public TalonFXWrapper driveMotor() {
        return driveMotor;
    }

    public CANcoder encoder() {
        return encoder;
    }

    protected MK4IModuleSignals signals() {
        return signals;
    }

}
