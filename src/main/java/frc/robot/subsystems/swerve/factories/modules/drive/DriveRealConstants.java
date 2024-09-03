package frc.robot.subsystems.swerve.factories.modules.drive;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.IDs;
import frc.robot.subsystems.swerve.modules.ModuleUtils;
import frc.robot.subsystems.swerve.modules.drive.talonfx.TalonFXDriveConstants;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

class DriveRealConstants {

	private static final boolean ENABLE_FOC = true;

	private static final double SLIP_CURRENT = 60;

	private static SysIdRoutine.Config generateSysidConfig() {
		return new SysIdRoutine.Config(
			Volts.of(0.5).per(Seconds.of(1)),
			Volts.of(2),
			null,
			state -> SignalLogger.writeString("state", state.toString())
		);
	}

	private static TalonFXConfiguration generateMotorConfiguration() {
		TalonFXConfiguration configuration = new TalonFXConfiguration();

		configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		configuration.Feedback.SensorToMechanismRatio = 6.12;

		configuration.TorqueCurrent.PeakForwardTorqueCurrent = SLIP_CURRENT;
		configuration.TorqueCurrent.PeakReverseTorqueCurrent = -SLIP_CURRENT;
		configuration.CurrentLimits.StatorCurrentLimit = SLIP_CURRENT;
		configuration.CurrentLimits.StatorCurrentLimitEnable = true;

		configuration.Slot0.kS = 0.21549;
		configuration.Slot0.kV = 0.72124;
		configuration.Slot0.kA = 0.11218;
		configuration.Slot0.kP = 1.5;
		configuration.Slot0.kI = 0;
		configuration.Slot0.kD = 0;

		return configuration;
	}

	protected static TalonFXDriveConstants FRONT_LEFT_CONSTANTS(String logPathPrefix) {
		return new TalonFXDriveConstants(
			IDs.TalonFXIDs.FRONT_LEFT_DRIVE_MOTOR,
			false,
			generateMotorConfiguration(),
			ENABLE_FOC,
			generateSysidConfig(),
			logPathPrefix + ModuleUtils.ModulePosition.FRONT_LEFT + "/"
		);
	}

	protected static TalonFXDriveConstants FRONT_RIGHT_CONSTANTS(String logPathPrefix) {
		return new TalonFXDriveConstants(
			IDs.TalonFXIDs.FRONT_RIGHT_DRIVE_MOTOR,
			true,
			generateMotorConfiguration(),
			ENABLE_FOC,
			generateSysidConfig(),
			logPathPrefix + ModuleUtils.ModulePosition.FRONT_RIGHT + "/"
		);
	}

	protected static TalonFXDriveConstants BACK_LEFT_CONSTANTS(String logPathPrefix) {
		return new TalonFXDriveConstants(
			IDs.TalonFXIDs.BACK_LEFT_DRIVE_MOTOR,
			false,
			generateMotorConfiguration(),
			ENABLE_FOC,
			generateSysidConfig(),
			logPathPrefix + ModuleUtils.ModulePosition.BACK_LEFT + "/"
		);
	}

	protected static TalonFXDriveConstants BACK_RIGHT_CONSTANTS(String logPathPrefix) {
		return new TalonFXDriveConstants(
			IDs.TalonFXIDs.BACK_RIGHT_DRIVE_MOTOR,
			false,
			generateMotorConfiguration(),
			ENABLE_FOC,
			generateSysidConfig(),
			logPathPrefix + ModuleUtils.ModulePosition.BACK_RIGHT + "/"
		);
	}

}
