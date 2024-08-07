package frc.robot.subsystems.swerve.factories.modules.drive;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.constants.IDs;
import frc.robot.subsystems.swerve.modules.drive.talonfx.TalonFXDriveConstants;

class DriveRealConstants {

	private static final boolean ENABLE_FOC = true;

	private static final double SLIP_CURRENT = 60;

	private static final TalonFXConfiguration MOTOR_CONFIG = new TalonFXConfiguration();
	static {
		MOTOR_CONFIG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		MOTOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		MOTOR_CONFIG.Feedback.SensorToMechanismRatio = 6.12;

		MOTOR_CONFIG.TorqueCurrent.PeakForwardTorqueCurrent = SLIP_CURRENT;
		MOTOR_CONFIG.TorqueCurrent.PeakReverseTorqueCurrent = -SLIP_CURRENT;
		MOTOR_CONFIG.CurrentLimits.StatorCurrentLimit = SLIP_CURRENT;
		MOTOR_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;

		MOTOR_CONFIG.Slot0.kS = 0.21549;
		MOTOR_CONFIG.Slot0.kV = 0.72124;
		MOTOR_CONFIG.Slot0.kA = 0.11218;
		MOTOR_CONFIG.Slot0.kP = 1.5;
		MOTOR_CONFIG.Slot0.kI = 0;
		MOTOR_CONFIG.Slot0.kD = 0;
	}

	protected static final TalonFXDriveConstants FRONT_LEFT_CONSTANTS = new TalonFXDriveConstants(
		IDs.TalonFXIDs.FRONT_LEFT_DRIVE_MOTOR,
		false,
		MOTOR_CONFIG,
		ENABLE_FOC
	);

	protected static final TalonFXDriveConstants FRONT_RIGHT_CONSTANTS = new TalonFXDriveConstants(
		IDs.TalonFXIDs.FRONT_RIGHT_DRIVE_MOTOR,
		true,
		MOTOR_CONFIG,
		ENABLE_FOC
	);

	protected static final TalonFXDriveConstants BACK_LEFT_CONSTANTS = new TalonFXDriveConstants(
		IDs.TalonFXIDs.BACK_LEFT_DRIVE_MOTOR,
		false,
		MOTOR_CONFIG,
		ENABLE_FOC
	);

	protected static final TalonFXDriveConstants BACK_RIGHT_CONSTANTS = new TalonFXDriveConstants(
		IDs.TalonFXIDs.BACK_RIGHT_DRIVE_MOTOR,
		false,
		MOTOR_CONFIG,
		ENABLE_FOC
	);

}
