package frc.robot.subsystems.swerve.factories.modules.drive;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.constants.IDs;
import frc.robot.subsystems.swerve.modules.drive.talonfx.TalonFXDriveConstants;

public class DriveRealConstants {

    private static final boolean ENABLE_FOC_DRIVE = true;

    private static final double DRIVE_SLIP_CURRENT = 30; //todo - calibrate

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

    protected static final TalonFXDriveConstants FRONT_LEFT_CONSTANTS = new TalonFXDriveConstants(
            IDs.TalonFXIDs.FRONT_LEFT_DRIVE_MOTOR,
            false,
            DRIVE_MOTOR_CONFIG,
            ENABLE_FOC_DRIVE
    );

    protected static final TalonFXDriveConstants FRONT_RIGHT_CONSTANTS = new TalonFXDriveConstants(
            IDs.TalonFXIDs.FRONT_RIGHT_DRIVE_MOTOR,
            true,
            DRIVE_MOTOR_CONFIG,
            ENABLE_FOC_DRIVE
    );

    protected static final TalonFXDriveConstants BACK_LEFT_CONSTANTS = new TalonFXDriveConstants(
            IDs.TalonFXIDs.BACK_LEFT_DRIVE_MOTOR,
            false,
            DRIVE_MOTOR_CONFIG,
            ENABLE_FOC_DRIVE
    );

    protected static final TalonFXDriveConstants BACK_RIGHT_CONSTANTS = new TalonFXDriveConstants(
            IDs.TalonFXIDs.BACK_RIGHT_DRIVE_MOTOR,
            false,
            DRIVE_MOTOR_CONFIG,
            ENABLE_FOC_DRIVE
    );

}
