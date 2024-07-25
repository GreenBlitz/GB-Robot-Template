package frc.robot.subsystems.swerve.constants;

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

class RealModulesConstants {

    protected static TalonFXModuleConstants[] getModulesConstants(double velocityAt12VoltsMetersPerSecond) {
        return new TalonFXModuleConstants[]{
                getTalonFXModuleConstants(
                        new ModuleID(
                                IDs.TalonFXIDs.FRONT_LEFT_STEER_MOTOR,
                                true,
                                IDs.TalonFXIDs.FRONT_LEFT_DRIVE_MOTOR,
                                false,
                                IDs.CANCodersIDs.FRONT_LEFT_ENCODER
                        ),
                        velocityAt12VoltsMetersPerSecond
                ),
                getTalonFXModuleConstants(
                        new ModuleID(
                                IDs.TalonFXIDs.FRONT_RIGHT_STEER_MOTOR,
                                true,
                                IDs.TalonFXIDs.FRONT_RIGHT_DRIVE_MOTOR,
                                true,
                                IDs.CANCodersIDs.FRONT_RIGHT_ENCODER
                        ),
                        velocityAt12VoltsMetersPerSecond
                ),
                getTalonFXModuleConstants(
                        new ModuleID(
                                IDs.TalonFXIDs.BACK_LEFT_STEER_MOTOR,
                                false,
                                IDs.TalonFXIDs.BACK_LEFT_DRIVE_MOTOR,
                                false,
                                IDs.CANCodersIDs.BACK_LEFT_ENCODER
                        ),
                        velocityAt12VoltsMetersPerSecond
                ),
                getTalonFXModuleConstants(
                        new ModuleID(
                                IDs.TalonFXIDs.BACK_RIGHT_STEER_MOTOR,
                                true,
                                IDs.TalonFXIDs.BACK_RIGHT_DRIVE_MOTOR,
                                false,
                                IDs.CANCodersIDs.BACK_RIGHT_ENCODER
                        ),
                        velocityAt12VoltsMetersPerSecond
                )
        };
    }

    private static TalonFXModuleConstants getTalonFXModuleConstants(ModuleID moduleID, double velocityAt12VoltsMetersPerSecond) {
        return new TalonFXModuleConstants(
                WHEEL_DIAMETER_METERS,
                COUPLING_RATIO,
                velocityAt12VoltsMetersPerSecond,
                ENABLE_FOC_STEER,
                ENABLE_FOC_DRIVE,
                STEER_MOTOR_CONFIG,
                DRIVE_MOTOR_CONFIG,
                ENCODER_CONFIG,
                moduleID
        );
    }

    private static final double WHEEL_DIAMETER_METERS = 0.048359 * 2;
    private static final double COUPLING_RATIO = 0.59;

    private static final boolean ENABLE_FOC_STEER = true;
    private static final boolean ENABLE_FOC_DRIVE = true;

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

        STEER_MOTOR_CONFIG.TorqueCurrent.PeakForwardTorqueCurrent = STEER_CURRENT_LIMIT;
        STEER_MOTOR_CONFIG.TorqueCurrent.PeakReverseTorqueCurrent = -STEER_CURRENT_LIMIT;
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

}
