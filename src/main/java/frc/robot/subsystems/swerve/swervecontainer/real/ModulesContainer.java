package frc.robot.subsystems.swerve.swervecontainer.real;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.robot.constants.IDs;
import frc.robot.subsystems.swerve.modules.ModuleID;
import frc.robot.subsystems.swerve.modules.talonfx.TalonFXModuleConstants;

public class ModulesContainer {

    private static final double DRIVE_SLIP_CURRENT = 30; //todo - calibrate
    private static final double STEER_CURRENT_LIMIT = 30; //todo - calibrate

    private static final CANcoderConfiguration ENCODER_CONFIG = new CANcoderConfiguration();
    static {
        ENCODER_CONFIG.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        ENCODER_CONFIG.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    }

    private static final TalonFXConfiguration DRIVE_MOTOR_CONFIG = new TalonFXConfiguration();
    static {
        DRIVE_MOTOR_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        DRIVE_MOTOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        DRIVE_MOTOR_CONFIG.Feedback.SensorToMechanismRatio = 6.12;

        DRIVE_MOTOR_CONFIG.TorqueCurrent.PeakForwardTorqueCurrent = DRIVE_SLIP_CURRENT;
        DRIVE_MOTOR_CONFIG.TorqueCurrent.PeakReverseTorqueCurrent = -DRIVE_SLIP_CURRENT;
        DRIVE_MOTOR_CONFIG.CurrentLimits.StatorCurrentLimit = DRIVE_SLIP_CURRENT;
        DRIVE_MOTOR_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;

        DRIVE_MOTOR_CONFIG.Slot0.kS = 0.21549;
        DRIVE_MOTOR_CONFIG.Slot0.kV = 0.72124;
        DRIVE_MOTOR_CONFIG.Slot0.kA = 0.11218;
        DRIVE_MOTOR_CONFIG.Slot0.kP = 1.5;
        DRIVE_MOTOR_CONFIG.Slot0.kI = 0;
        DRIVE_MOTOR_CONFIG.Slot0.kD = 0;
    }

    private static final TalonFXConfiguration STEER_MOTOR_CONFIG = new TalonFXConfiguration();
    static {
        STEER_MOTOR_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        STEER_MOTOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        STEER_MOTOR_CONFIG.CurrentLimits.StatorCurrentLimit = STEER_CURRENT_LIMIT;
        STEER_MOTOR_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;

        STEER_MOTOR_CONFIG.Feedback.RotorToSensorRatio = 150.0 / 7.0;
        STEER_MOTOR_CONFIG.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

        STEER_MOTOR_CONFIG.Slot0.kS = 0.19648;
        STEER_MOTOR_CONFIG.Slot0.kV = 2.5763;
        STEER_MOTOR_CONFIG.Slot0.kA = 0.50361;
        STEER_MOTOR_CONFIG.Slot0.kP = 88;
        STEER_MOTOR_CONFIG.Slot0.kI = 0;
        STEER_MOTOR_CONFIG.Slot0.kD = 1.5;
        STEER_MOTOR_CONFIG.ClosedLoopGeneral.ContinuousWrap = true;
    }

    private static TalonFXModuleConstants getMk4iModuleConstants(ModuleID moduleID) {
        return new TalonFXModuleConstants(
                0.048359 * 2,
                0.59,
                RealSwerve.VELOCITY_AT_12_VOLTS_METERS_PER_SECOND,
                true,
                true,
                STEER_MOTOR_CONFIG,
                DRIVE_MOTOR_CONFIG,
                ENCODER_CONFIG,
                moduleID
        );
    }


    protected static final TalonFXModuleConstants[] moduleConstants = {
            getMk4iModuleConstants(new ModuleID(
                    IDs.TalonFXIDs.FRONT_LEFT_STEER_MOTOR,
                    true,
                    IDs.TalonFXIDs.FRONT_LEFT_DRIVE_MOTOR,
                    false,
                    IDs.CANCodersIDs.FRONT_LEFT_ENCODER
            )),
            getMk4iModuleConstants(new ModuleID(
                    IDs.TalonFXIDs.FRONT_RIGHT_STEER_MOTOR,
                    true,
                    IDs.TalonFXIDs.FRONT_RIGHT_DRIVE_MOTOR,
                    true,
                    IDs.CANCodersIDs.FRONT_RIGHT_ENCODER
            )),
            getMk4iModuleConstants(new ModuleID(
                    IDs.TalonFXIDs.BACK_LEFT_STEER_MOTOR,
                    false,
                    IDs.TalonFXIDs.BACK_LEFT_DRIVE_MOTOR,
                    false,
                    IDs.CANCodersIDs.BACK_LEFT_ENCODER
            )),
            getMk4iModuleConstants(new ModuleID(
                    IDs.TalonFXIDs.BACK_RIGHT_STEER_MOTOR,
                    true,
                    IDs.TalonFXIDs.BACK_RIGHT_DRIVE_MOTOR,
                    false,
                    IDs.CANCodersIDs.BACK_RIGHT_ENCODER
            ))
    };

}
