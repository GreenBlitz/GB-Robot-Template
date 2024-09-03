package frc.robot.subsystems.swerve.factories.modules.steer;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.IDs;
import frc.robot.subsystems.swerve.modules.ModuleUtils;
import frc.robot.subsystems.swerve.modules.steer.talonfx.TalonFXSteerConstants;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

class SteerRealConstants {

	private static final boolean ENABLE_FOC = true;

	private static SysIdRoutine.Config generateSysidConfig() {
		return new SysIdRoutine.Config(
			Volts.of(0.5).per(Seconds.of(1)),
			Volts.of(1),
			null,
			state -> SignalLogger.writeString("state", state.toString())
		);
	}

	private static TalonFXConfiguration generateMotorConfiguration() {
		TalonFXConfiguration configuration = new TalonFXConfiguration();

		configuration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		configuration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		configuration.CurrentLimits.StatorCurrentLimit = 30;
		configuration.CurrentLimits.StatorCurrentLimitEnable = true;

		configuration.Feedback.RotorToSensorRatio = 150.0 / 7.0;
		configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

		configuration.Slot0.kS = 0.19648;
		configuration.Slot0.kV = 2.5763;
		configuration.Slot0.kA = 0.50361;
		configuration.Slot0.kP = 88;
		configuration.Slot0.kI = 0;
		configuration.Slot0.kD = 1.5;
		configuration.ClosedLoopGeneral.ContinuousWrap = true;

		return configuration;
	}

	protected static TalonFXSteerConstants FRONT_LEFT_CONSTANTS(String logPathPrefix) {
		return new TalonFXSteerConstants(
			IDs.TalonFXIDs.FRONT_LEFT_STEER_MOTOR,
			true,
			IDs.CANCodersIDs.FRONT_LEFT_ENCODER.ID(),
			generateMotorConfiguration(),
			ENABLE_FOC,
			generateSysidConfig(),
			logPathPrefix + ModuleUtils.ModulePosition.FRONT_LEFT + "/"
		);
	}

	protected static TalonFXSteerConstants FRONT_RIGHT_CONSTANTS(String logPathPrefix) {
		return new TalonFXSteerConstants(
			IDs.TalonFXIDs.FRONT_RIGHT_STEER_MOTOR,
			true,
			IDs.CANCodersIDs.FRONT_RIGHT_ENCODER.ID(),
			generateMotorConfiguration(),
			ENABLE_FOC,
			generateSysidConfig(),
			logPathPrefix + ModuleUtils.ModulePosition.FRONT_RIGHT + "/"
		);
	}

	protected static TalonFXSteerConstants BACK_LEFT_CONSTANTS(String logPathPrefix) {
		return new TalonFXSteerConstants(
			IDs.TalonFXIDs.BACK_LEFT_STEER_MOTOR,
			false,
			IDs.CANCodersIDs.BACK_LEFT_ENCODER.ID(),
			generateMotorConfiguration(),
			ENABLE_FOC,
			generateSysidConfig(),
			logPathPrefix + ModuleUtils.ModulePosition.BACK_LEFT + "/"
		);
	}

	protected static TalonFXSteerConstants BACK_RIGHT_CONSTANTS(String logPathPrefix) {
		return new TalonFXSteerConstants(
			IDs.TalonFXIDs.BACK_RIGHT_STEER_MOTOR,
			true,
			IDs.CANCodersIDs.BACK_RIGHT_ENCODER.ID(),
			generateMotorConfiguration(),
			ENABLE_FOC,
			generateSysidConfig(),
			logPathPrefix + ModuleUtils.ModulePosition.BACK_RIGHT + "/"
		);
	}

}
