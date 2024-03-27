package frc.robot.subsystems.swerve.mk4iswerve.mk4imodule;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.*;
import frc.robot.constants.Phoenix6Constants;
import frc.robot.constants.Ports;
import frc.robot.subsystems.swerve.ModuleConstants;

public class MK4IModuleConstants {

    public static final boolean ENABLE_FOC_DRIVE = true;
    public static final boolean ENABLE_FOC_STEER = true;

    public static final double COUPLING_RATIO = 0;


    private static final InvertedValue DRIVE_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive;
    private static final InvertedValue STEER_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;


    private static final SensorDirectionValue STEER_ENCODER_DIRECTION = SensorDirectionValue.Clockwise_Positive;
    private static final AbsoluteSensorRangeValue STEER_ENCODER_RANGE = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;


    private static final NeutralModeValue DRIVE_MOTOR_NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    private static final NeutralModeValue STEER_MOTOR_NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;


    private static final double DRIVE_SLIP_CURRENT = 100;
    private static final double STEER_CURRENT_LIMIT = 50;


    private static final double STEER_MOTOR_P = 20;
    private static final double STEER_MOTOR_I = 0;
    private static final double STEER_MOTOR_D = 0;


    private static final double DRIVE_MOTOR_P = 3;
    private static final double DRIVE_MOTOR_I = 0;
    private static final double DRIVE_MOTOR_D = 0;

    protected static final CANcoderConfiguration ENCODER_CONFIG = new CANcoderConfiguration();
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

        
        DRIVE_MOTOR_CONFIG.Slot0.kS = 0.009;
        DRIVE_MOTOR_CONFIG.Slot0.kA = 0.22448;
        DRIVE_MOTOR_CONFIG.Slot0.kV = 0.71632;
        DRIVE_MOTOR_CONFIG.Slot0.kP = DRIVE_MOTOR_P;
        DRIVE_MOTOR_CONFIG.Slot0.kI = DRIVE_MOTOR_I;
        DRIVE_MOTOR_CONFIG.Slot0.kD = DRIVE_MOTOR_D;
    }

    protected static final TalonFXConfiguration STEER_MOTOR_CONFIG = new TalonFXConfiguration();
    static {
        STEER_MOTOR_CONFIG.MotorOutput.Inverted = STEER_MOTOR_INVERTED_VALUE;
        STEER_MOTOR_CONFIG.MotorOutput.NeutralMode = STEER_MOTOR_NEUTRAL_MODE_VALUE;
        STEER_MOTOR_CONFIG.CurrentLimits.StatorCurrentLimit = STEER_CURRENT_LIMIT;
        STEER_MOTOR_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;

        STEER_MOTOR_CONFIG.Feedback.SensorToMechanismRatio = ModuleConstants.STEER_GEAR_RATIO;
        STEER_MOTOR_CONFIG.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        STEER_MOTOR_CONFIG.Slot0.kS = 0.32;
        STEER_MOTOR_CONFIG.Slot0.kA = 0;
        STEER_MOTOR_CONFIG.Slot0.kV = 0;
        STEER_MOTOR_CONFIG.Slot0.kP = STEER_MOTOR_P;
        STEER_MOTOR_CONFIG.Slot0.kI = STEER_MOTOR_I;
        STEER_MOTOR_CONFIG.Slot0.kD = STEER_MOTOR_D;
        STEER_MOTOR_CONFIG.ClosedLoopGeneral.ContinuousWrap = true;
    }
    
    
    protected static final MK4IModuleConfigObject FRONT_LEFT = new MK4IModuleConfigObject(
            Phoenix6Constants.CANIVORE_NAME,
            Ports.TalonFXIds.FRONT_LEFT_STEER_MOTOR,
            true,
            Ports.TalonFXIds.FRONT_LEFT_DRIVE_MOTOR,
            false,
            Ports.CANCodersIds.FRONT_LEFT_ENCODER
    );
    
    protected static final MK4IModuleConfigObject FRONT_RIGHT = new MK4IModuleConfigObject(
            Phoenix6Constants.CANIVORE_NAME,
            Ports.TalonFXIds.FRONT_RIGHT_STEER_MOTOR,
            true,
            Ports.TalonFXIds.FRONT_RIGHT_DRIVE_MOTOR,
            true,
            Ports.CANCodersIds.FRONT_RIGHT_ENCODER
    );
    
    protected static final MK4IModuleConfigObject BACK_LEFT = new MK4IModuleConfigObject(
            Phoenix6Constants.CANIVORE_NAME,
            Ports.TalonFXIds.BACK_LEFT_STEER_MOTOR,
            false,
            Ports.TalonFXIds.BACK_LEFT_DRIVE_MOTOR,
            false,
            Ports.CANCodersIds.BACK_LEFT_ENCODER
    );
    
    protected static final MK4IModuleConfigObject BACK_RIGHT = new MK4IModuleConfigObject(
            Phoenix6Constants.CANIVORE_NAME,
            Ports.TalonFXIds.BACK_RIGHT_STEER_MOTOR,
            true,
            Ports.TalonFXIds.BACK_RIGHT_DRIVE_MOTOR,
            false,
            Ports.CANCodersIds.BACK_RIGHT_ENCODER
    );

}
