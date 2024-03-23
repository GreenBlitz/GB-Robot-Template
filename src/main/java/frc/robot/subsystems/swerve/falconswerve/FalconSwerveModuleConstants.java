package frc.robot.subsystems.swerve.falconswerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.*;
import frc.robot.constants.Phoenix6Constants;
import frc.utils.Conversions;
import frc.utils.RobotTypeUtils;
import frc.utils.devicewrappers.GBCANCoder;
import frc.utils.devicewrappers.GBTalonFXPro;

public class FalconSwerveModuleConstants {

    public static final double WHEEL_DIAMETER_METERS = 0.1016;
    public static final double MAX_SPEED_REVOLUTIONS_PER_SECOND = Conversions.distanceToRevolutions(FalconSwerveConstants.MAX_SPEED_METERS_PER_SECOND, WHEEL_DIAMETER_METERS);
    public static final double VOLTAGE_COMPENSATION_SATURATION = 12;
    public static final boolean ENABLE_FOC = true;

    public static final double
            DRIVE_GEAR_RATIO = 6.75,
            STEER_GEAR_RATIO = 12.8,
            COUPLING_RATIO = 0;

    public static final int
            FRONT_LEFT_ID = 0,
            FRONT_RIGHT_ID = 1,
            BACK_LEFT_ID = 2,
            BACK_RIGHT_ID = 3;
    private static final int
            FRONT_LEFT_DRIVE_MOTOR_ID = FRONT_LEFT_ID + 1,
            FRONT_RIGHT_DRIVE_MOTOR_ID = FRONT_RIGHT_ID + 1,
            BACK_LEFT_DRIVE_MOTOR_ID = BACK_LEFT_ID + 1,
            BACK_RIGHT_DRIVE_MOTOR_ID = BACK_RIGHT_ID + 1;
    private static final int
            FRONT_LEFT_STEER_MOTOR_ID = FRONT_LEFT_ID + 2,
            FRONT_RIGHT_STEER_MOTOR_ID = FRONT_RIGHT_ID + 2,
            BACK_LEFT_STEER_MOTOR_ID = BACK_LEFT_ID + 2,
            BACK_RIGHT_STEER_MOTOR_ID = BACK_RIGHT_ID + 2;

    private static final InvertedValue
            DRIVE_MOTOR_INVERTED_VALUE = InvertedValue.Clockwise_Positive,
            STEER_MOTOR_INVERTED_VALUE = InvertedValue.CounterClockwise_Positive;
    private static final SensorDirectionValue STEER_ENCODER_DIRECTION = SensorDirectionValue.Clockwise_Positive;
    private static final AbsoluteSensorRangeValue STEER_ENCODER_RANGE = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    private static final NeutralModeValue
            DRIVE_MOTOR_NEUTRAL_MODE_VALUE = NeutralModeValue.Brake,
            STEER_MOTOR_NEUTRAL_MODE_VALUE = NeutralModeValue.Brake;
    private static final double
            DRIVE_SLIP_CURRENT = 100,
            STEER_CURRENT_LIMIT = 50;

    private static final double
            STEER_MOTOR_P = 75,
            STEER_MOTOR_I = 0,
            STEER_MOTOR_D = 0;
    private static final double
            DRIVE_MOTOR_P = 40,
            DRIVE_MOTOR_I = 0,
            DRIVE_MOTOR_D = 0;

    private static final GBTalonFXPro
            FRONT_LEFT_DRIVE_MOTOR = new GBTalonFXPro(FRONT_LEFT_DRIVE_MOTOR_ID, Phoenix6Constants.CANIVORE_NAME),
            FRONT_RIGHT_DRIVE_MOTOR = new GBTalonFXPro(FRONT_RIGHT_DRIVE_MOTOR_ID, Phoenix6Constants.CANIVORE_NAME),
            BACK_LEFT_DRIVE_MOTOR = new GBTalonFXPro(BACK_LEFT_DRIVE_MOTOR_ID, Phoenix6Constants.CANIVORE_NAME),
            BACK_RIGHT_DRIVE_MOTOR = new GBTalonFXPro(BACK_RIGHT_DRIVE_MOTOR_ID, Phoenix6Constants.CANIVORE_NAME);
    private static final GBTalonFXPro
            FRONT_LEFT_STEER_MOTOR = new GBTalonFXPro(FRONT_LEFT_STEER_MOTOR_ID, Phoenix6Constants.CANIVORE_NAME),
            FRONT_RIGHT_STEER_MOTOR = new GBTalonFXPro(FRONT_RIGHT_STEER_MOTOR_ID, Phoenix6Constants.CANIVORE_NAME),
            BACK_LEFT_STEER_MOTOR = new GBTalonFXPro(BACK_LEFT_STEER_MOTOR_ID, Phoenix6Constants.CANIVORE_NAME),
            BACK_RIGHT_STEER_MOTOR = new GBTalonFXPro(BACK_RIGHT_STEER_MOTOR_ID, Phoenix6Constants.CANIVORE_NAME);
    private static final GBCANCoder
            FRONT_LEFT_STEER_ENCODER = new GBCANCoder(FRONT_LEFT_ID, Phoenix6Constants.CANIVORE_NAME),
            FRONT_RIGHT_STEER_ENCODER = new GBCANCoder(FRONT_LEFT_ID, Phoenix6Constants.CANIVORE_NAME),
            BACK_LEFT_STEER_ENCODER = new GBCANCoder(FRONT_LEFT_ID, Phoenix6Constants.CANIVORE_NAME),
            BACK_RIGHT_STEER_ENCODER = new GBCANCoder(FRONT_LEFT_ID, Phoenix6Constants.CANIVORE_NAME);

    public static final FalconSwerveModuleConstants
            FRONT_LEFT_SWERVE_MODULE_CONSTANTS = new FalconSwerveModuleConstants(
                FRONT_LEFT_DRIVE_MOTOR,
                FRONT_LEFT_STEER_MOTOR,
                FRONT_LEFT_STEER_ENCODER
            ),
            FRONT_RIGHT_SWERVE_MODULE_CONSTANTS = new FalconSwerveModuleConstants(
                    FRONT_RIGHT_DRIVE_MOTOR,
                    FRONT_RIGHT_STEER_MOTOR,
                    FRONT_RIGHT_STEER_ENCODER
            ),
            BACK_LEFT_SWERVE_MODULE_CONSTANTS = new FalconSwerveModuleConstants(
                    BACK_LEFT_DRIVE_MOTOR,
                    BACK_LEFT_STEER_MOTOR,
                    BACK_LEFT_STEER_ENCODER
            ),
            BACK_RIGHT_SWERVE_MODULE_CONSTANTS = new FalconSwerveModuleConstants(
                    BACK_RIGHT_DRIVE_MOTOR,
                    BACK_RIGHT_STEER_MOTOR,
                    BACK_RIGHT_STEER_ENCODER
            );

    final GBTalonFXPro driveMotor, steerMotor;
    final GBCANCoder steerEncoder;
    StatusSignal<Double> steerPositionSignal, steerVelocitySignal, steerVoltageSignal, driveStatorCurrentSignal, drivePositionSignal, driveVelocitySignal, driveVoltageSignal;

    private FalconSwerveModuleConstants(GBTalonFXPro driveMotor, GBTalonFXPro steerMotor, GBCANCoder steerEncoder) {
        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;
        this.steerEncoder = steerEncoder;

        if (!RobotTypeUtils.isReplay()) {
            configureSteerEncoder();
            configureDriveMotor();
            configureSteerMotor();
        }
    }

    private void configureSteerEncoder() {
        final CANcoderConfiguration config = new CANcoderConfiguration();

        config.MagnetSensor.SensorDirection = STEER_ENCODER_DIRECTION;
        config.MagnetSensor.AbsoluteSensorRange = STEER_ENCODER_RANGE;

        steerEncoder.applyConfiguration(config);

        steerPositionSignal = steerMotor.getPosition();
        steerVelocitySignal = steerMotor.getVelocity();
        steerVoltageSignal = steerMotor.getMotorVoltage();

        steerEncoder.updateFrequency(
                Phoenix6Constants.FAST_SIGNAL_FREQUENCY_HERTZ,
                steerPositionSignal,
                steerVelocitySignal,
                steerVoltageSignal
        );

        steerEncoder.optimizeBusUtilization();
    }

    private void configureDriveMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = DRIVE_MOTOR_INVERTED_VALUE;
        config.MotorOutput.NeutralMode = DRIVE_MOTOR_NEUTRAL_MODE_VALUE;
        config.Feedback.SensorToMechanismRatio = DRIVE_GEAR_RATIO;

        config.TorqueCurrent.PeakForwardTorqueCurrent = DRIVE_SLIP_CURRENT;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -DRIVE_SLIP_CURRENT;
        config.CurrentLimits.StatorCurrentLimit = DRIVE_SLIP_CURRENT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.Slot0.kP = DRIVE_MOTOR_P;
        config.Slot0.kI = DRIVE_MOTOR_I;
        config.Slot0.kD = DRIVE_MOTOR_D;

        driveMotor.applyConfiguration(config);

        drivePositionSignal = driveMotor.getPosition();
        driveVelocitySignal = driveMotor.getVelocity();
        driveStatorCurrentSignal = driveMotor.getStatorCurrent();
        driveVoltageSignal = driveMotor.getMotorVoltage();

        driveMotor.updateFrequency(
                Phoenix6Constants.FAST_SIGNAL_FREQUENCY_HERTZ,
                drivePositionSignal,
                driveVelocitySignal
        );
        driveMotor.updateFrequency(
                Phoenix6Constants.NORMAL_SIGNAL_FREQUENCY_HERTZ,
                driveVoltageSignal,
                driveStatorCurrentSignal
        );

        driveMotor.optimizeBusUtilization();
    }

    private void configureSteerMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = STEER_MOTOR_INVERTED_VALUE;
        config.MotorOutput.NeutralMode = STEER_MOTOR_NEUTRAL_MODE_VALUE;
        config.CurrentLimits.StatorCurrentLimit = STEER_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.Feedback.RotorToSensorRatio = STEER_GEAR_RATIO;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        config.Feedback.FeedbackRemoteSensorID = steerEncoder.getDeviceID();

        config.Slot0.kP = STEER_MOTOR_P;
        config.Slot0.kI = STEER_MOTOR_I;
        config.Slot0.kD = STEER_MOTOR_D;
        config.ClosedLoopGeneral.ContinuousWrap = true;

        steerMotor.applyConfiguration(config);

        steerPositionSignal = steerMotor.getPosition();
        steerVelocitySignal = steerMotor.getVelocity();
        steerVoltageSignal = steerMotor.getMotorVoltage();

        steerMotor.updateFrequency(
                Phoenix6Constants.FAST_SIGNAL_FREQUENCY_HERTZ,
                steerPositionSignal,
                steerVelocitySignal
        );
        steerMotor.updateFrequency(
                Phoenix6Constants.SLOW_SIGNAL_FREQUENCY_HERTZ,
                steerVoltageSignal
        );

        steerMotor.optimizeBusUtilization();
    }

}
