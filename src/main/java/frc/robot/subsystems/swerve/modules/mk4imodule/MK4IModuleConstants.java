package frc.robot.subsystems.swerve.modules.mk4imodule;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.robot.constants.Ports;
import frc.robot.subsystems.swerve.modules.ModuleConstants;

class MK4IModuleConstants {

    public static final double COUPLING_RATIO = 0.59;

    private static final InvertedValue DRIVE_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive;
    private static final InvertedValue STEER_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;

    private static final SensorDirectionValue STEER_ENCODER_DIRECTION = SensorDirectionValue.CounterClockwise_Positive;
    private static final AbsoluteSensorRangeValue STEER_ENCODER_RANGE = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

    private static final NeutralModeValue DRIVE_MOTOR_NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    private static final NeutralModeValue STEER_MOTOR_NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;

    private static final double DRIVE_SLIP_CURRENT = 100; //todo - return to 100
    private static final double STEER_CURRENT_LIMIT = 40;//todo - return to 100

    protected static final CANcoderConfiguration ENCODER_CONFIG = new CANcoderConfiguration();//todo - calibrate offsets
    static {
        ENCODER_CONFIG.MagnetSensor.SensorDirection = STEER_ENCODER_DIRECTION;
        ENCODER_CONFIG.MagnetSensor.AbsoluteSensorRange = STEER_ENCODER_RANGE;
    }

    protected static final TalonFXConfiguration DRIVE_MOTOR_CONFIG = new TalonFXConfiguration();
    static {
        DRIVE_MOTOR_CONFIG.MotorOutput.Inverted = DRIVE_MOTOR_INVERTED_VALUE;
        DRIVE_MOTOR_CONFIG.MotorOutput.NeutralMode = DRIVE_MOTOR_NEUTRAL_MODE_VALUE;
        DRIVE_MOTOR_CONFIG.Feedback.SensorToMechanismRatio = ModuleConstants.DRIVE_GEAR_RATIO;

        DRIVE_MOTOR_CONFIG.TorqueCurrent.PeakForwardTorqueCurrent = DRIVE_SLIP_CURRENT;
        DRIVE_MOTOR_CONFIG.TorqueCurrent.PeakReverseTorqueCurrent = -DRIVE_SLIP_CURRENT;
        DRIVE_MOTOR_CONFIG.CurrentLimits.StatorCurrentLimit = DRIVE_SLIP_CURRENT;
        DRIVE_MOTOR_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;

        DRIVE_MOTOR_CONFIG.Slot0.kS = 0.21549;
        DRIVE_MOTOR_CONFIG.Slot0.kV = 0.72124;
        DRIVE_MOTOR_CONFIG.Slot0.kA = 0.11218;
        DRIVE_MOTOR_CONFIG.Slot0.kP = 0;
        DRIVE_MOTOR_CONFIG.Slot0.kI = 0;
        DRIVE_MOTOR_CONFIG.Slot0.kD = 0;
    }

    protected static final TalonFXConfiguration STEER_MOTOR_CONFIG = new TalonFXConfiguration();
    static {
        STEER_MOTOR_CONFIG.MotorOutput.Inverted = STEER_MOTOR_INVERTED_VALUE;
        STEER_MOTOR_CONFIG.MotorOutput.NeutralMode = STEER_MOTOR_NEUTRAL_MODE_VALUE;
        STEER_MOTOR_CONFIG.CurrentLimits.StatorCurrentLimit = STEER_CURRENT_LIMIT;
        STEER_MOTOR_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;

        STEER_MOTOR_CONFIG.Feedback.SensorToMechanismRatio = ModuleConstants.STEER_GEAR_RATIO;
        STEER_MOTOR_CONFIG.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        STEER_MOTOR_CONFIG.Slot0.kS = 0.19648;
        STEER_MOTOR_CONFIG.Slot0.kV = 2.5763;
        STEER_MOTOR_CONFIG.Slot0.kA = 0.50361;
        STEER_MOTOR_CONFIG.Slot0.kP = 88;
        STEER_MOTOR_CONFIG.Slot0.kI = 0;
        STEER_MOTOR_CONFIG.Slot0.kD = 1.5;
        STEER_MOTOR_CONFIG.ClosedLoopGeneral.ContinuousWrap = true;
    }

    protected static final MK4IModuleConfigObject FRONT_LEFT = new MK4IModuleConfigObject(
            Ports.TalonFXIDs.FRONT_LEFT_STEER_MOTOR,
            true,
            Ports.TalonFXIDs.FRONT_LEFT_DRIVE_MOTOR,
            false,
            Ports.CANCodersIDs.FRONT_LEFT_ENCODER
    );

    protected static final MK4IModuleConfigObject FRONT_RIGHT = new MK4IModuleConfigObject(
            Ports.TalonFXIDs.FRONT_RIGHT_STEER_MOTOR,
            true,
            Ports.TalonFXIDs.FRONT_RIGHT_DRIVE_MOTOR,
            true,
            Ports.CANCodersIDs.FRONT_RIGHT_ENCODER
    );

    protected static final MK4IModuleConfigObject BACK_LEFT = new MK4IModuleConfigObject(
            Ports.TalonFXIDs.BACK_LEFT_STEER_MOTOR,
            false,
            Ports.TalonFXIDs.BACK_LEFT_DRIVE_MOTOR,
            false,
            Ports.CANCodersIDs.BACK_LEFT_ENCODER
    );

    protected static final MK4IModuleConfigObject BACK_RIGHT = new MK4IModuleConfigObject(
            Ports.TalonFXIDs.BACK_RIGHT_STEER_MOTOR,
            true,
            Ports.TalonFXIDs.BACK_RIGHT_DRIVE_MOTOR,
            false,
            Ports.CANCodersIDs.BACK_RIGHT_ENCODER
    );

}
