package frc.utils;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.robot.constants.SwerveConstants;

public final class CTREConfigs {

    public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

    public CTREConfigs(){
        this(InvertedValue.CounterClockwise_Positive,InvertedValue.CounterClockwise_Positive,SensorDirectionValue.CounterClockwise_Positive);
    }

    public CTREConfigs(InvertedValue driveInverted, InvertedValue angleInverted, SensorDirectionValue canCoderInverted){
        /** Swerve CANCoder Configuration */
        swerveCANcoderConfig.MagnetSensor.SensorDirection = canCoderInverted;

        /** Swerve Angle Motor Configurations */
        /* Motor Inverts and Neutral Mode */
        swerveAngleFXConfig.MotorOutput.Inverted = angleInverted;
        swerveAngleFXConfig.MotorOutput.NeutralMode = SwerveConstants.AngleConstants.NEUTRAL_MODE;

        /* Gear Ratio and Wrapping Config */
        swerveAngleFXConfig.Feedback.SensorToMechanismRatio = SwerveConstants.AngleConstants.GEAR_RATIO;
        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;

        /* Current Limiting */
        swerveAngleFXConfig.CurrentLimits = SwerveConstants.AngleConstants.CURRENT_LIMIT;

        /* PID Config */
        swerveAngleFXConfig.Slot0.kP = SwerveConstants.AngleConstants.PID.getP();
        swerveAngleFXConfig.Slot0.kI = SwerveConstants.AngleConstants.PID.getI();
        swerveAngleFXConfig.Slot0.kD = SwerveConstants.AngleConstants.PID.getD();

        /** Swerve Drive Motor Configuration */
        /* Motor Inverts and Neutral Mode */
        swerveDriveFXConfig.MotorOutput.Inverted = driveInverted;
        swerveDriveFXConfig.MotorOutput.NeutralMode = SwerveConstants.DriveConstants.NEUTRAL_MODE;

        /* Gear Ratio Config */
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = SwerveConstants.DriveConstants.GEAR_RATIO;

        /* Current Limiting */
        swerveDriveFXConfig.CurrentLimits = SwerveConstants.DriveConstants.CURRENT_LIMIT;

        /* PID Config */
        swerveDriveFXConfig.Slot0.kP = SwerveConstants.DriveConstants.PID.getP();
        swerveDriveFXConfig.Slot0.kI = SwerveConstants.DriveConstants.PID.getI();
        swerveDriveFXConfig.Slot0.kD = SwerveConstants.DriveConstants.PID.getD();

        /* Open and Closed Loop Ramping */
        swerveDriveFXConfig.OpenLoopRamps = SwerveConstants.DriveConstants.OPEN_LOOP_RAMPS_CONFIGS;
        swerveDriveFXConfig.ClosedLoopRamps = SwerveConstants.DriveConstants.CLOSED_LOOP_RAMPS_CONFIGS;
    }
}