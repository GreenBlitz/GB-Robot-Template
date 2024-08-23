package frc.robot.subsystems.swerve.factories.modules.drive;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.IDs;
import frc.robot.subsystems.swerve.modules.ModuleUtils;
import frc.robot.subsystems.swerve.modules.drive.talonfx.TalonFXDriveConstants;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

class DriveRealConstants {

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

	private static final Measure<Velocity<Voltage>> SYSID_RAMP_RATE = Volts.of(0.5).per(Seconds.of(1));
	private static final Measure<Voltage> SYSID_VOLTAGE_STEP = Volts.of(2);

	protected static TalonFXDriveConstants FRONT_LEFT_CONSTANTS(String logPathPrefix) {
		return new TalonFXDriveConstants(
			IDs.TalonFXIDs.FRONT_LEFT_DRIVE_MOTOR,
			false,
			MOTOR_CONFIG,
			new SysIdRoutine.Config(SYSID_RAMP_RATE, SYSID_VOLTAGE_STEP, null, (state) -> SignalLogger.writeString("state", state.toString())),
			logPathPrefix + ModuleUtils.ModulePosition.FRONT_LEFT + "/"
		);
	}

	protected static TalonFXDriveConstants FRONT_RIGHT_CONSTANTS(String logPathPrefix) {
		return new TalonFXDriveConstants(
			IDs.TalonFXIDs.FRONT_RIGHT_DRIVE_MOTOR,
			true,
			MOTOR_CONFIG,
			new SysIdRoutine.Config(SYSID_RAMP_RATE, SYSID_VOLTAGE_STEP, null, (state) -> SignalLogger.writeString("state", state.toString())),
			logPathPrefix + ModuleUtils.ModulePosition.FRONT_RIGHT + "/"
		);
	}

	protected static TalonFXDriveConstants BACK_LEFT_CONSTANTS(String logPathPrefix) {
		return new TalonFXDriveConstants(
			IDs.TalonFXIDs.BACK_LEFT_DRIVE_MOTOR,
			false,
			MOTOR_CONFIG,
			new SysIdRoutine.Config(SYSID_RAMP_RATE, SYSID_VOLTAGE_STEP, null, (state) -> SignalLogger.writeString("state", state.toString())),
			logPathPrefix + ModuleUtils.ModulePosition.BACK_LEFT + "/"
		);
	}

	protected static TalonFXDriveConstants BACK_RIGHT_CONSTANTS(String logPathPrefix) {
		return new TalonFXDriveConstants(
			IDs.TalonFXIDs.BACK_RIGHT_DRIVE_MOTOR,
			false,
			MOTOR_CONFIG,
			new SysIdRoutine.Config(SYSID_RAMP_RATE, SYSID_VOLTAGE_STEP, null, (state) -> SignalLogger.writeString("state", state.toString())),
			logPathPrefix + ModuleUtils.ModulePosition.BACK_RIGHT + "/"
		);
	}

}
