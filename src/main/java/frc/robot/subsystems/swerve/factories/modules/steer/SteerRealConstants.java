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

	private static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
		Volts.of(0.5).per(Seconds.of(1)),
		Volts.of(1),
		null,
		state -> SignalLogger.writeString("state", state.toString())
	);

	private static final TalonFXConfiguration MOTOR_CONFIG = new TalonFXConfiguration();
	static {
		MOTOR_CONFIG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		MOTOR_CONFIG.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		MOTOR_CONFIG.CurrentLimits.StatorCurrentLimit = 30;
		MOTOR_CONFIG.CurrentLimits.StatorCurrentLimitEnable = true;

		MOTOR_CONFIG.Feedback.RotorToSensorRatio = 150.0 / 7.0;
		MOTOR_CONFIG.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

		MOTOR_CONFIG.Slot0.kS = 0.19648;
		MOTOR_CONFIG.Slot0.kV = 2.5763;
		MOTOR_CONFIG.Slot0.kA = 0.50361;
		MOTOR_CONFIG.Slot0.kP = 88;
		MOTOR_CONFIG.Slot0.kI = 0;
		MOTOR_CONFIG.Slot0.kD = 1.5;
		MOTOR_CONFIG.ClosedLoopGeneral.ContinuousWrap = true;
	}

	protected static TalonFXSteerConstants FRONT_LEFT_CONSTANTS(String logPathPrefix) {
		return new TalonFXSteerConstants(
			IDs.TalonFXIDs.FRONT_LEFT_STEER_MOTOR,
			true,
			IDs.CANCodersIDs.FRONT_LEFT_ENCODER.ID(),
			MOTOR_CONFIG,
			ENABLE_FOC,
			SYSID_CONFIG,
			logPathPrefix + ModuleUtils.ModulePosition.FRONT_LEFT + "/"
		);
	}

	protected static TalonFXSteerConstants FRONT_RIGHT_CONSTANTS(String logPathPrefix) {
		return new TalonFXSteerConstants(
			IDs.TalonFXIDs.FRONT_RIGHT_STEER_MOTOR,
			true,
			IDs.CANCodersIDs.FRONT_RIGHT_ENCODER.ID(),
			MOTOR_CONFIG,
			ENABLE_FOC,
			SYSID_CONFIG,
			logPathPrefix + ModuleUtils.ModulePosition.FRONT_RIGHT + "/"
		);
	}

	protected static TalonFXSteerConstants BACK_LEFT_CONSTANTS(String logPathPrefix) {
		return new TalonFXSteerConstants(
			IDs.TalonFXIDs.BACK_LEFT_STEER_MOTOR,
			false,
			IDs.CANCodersIDs.BACK_LEFT_ENCODER.ID(),
			MOTOR_CONFIG,
			ENABLE_FOC,
			SYSID_CONFIG,
			logPathPrefix + ModuleUtils.ModulePosition.BACK_LEFT + "/"
		);
	}

	protected static TalonFXSteerConstants BACK_RIGHT_CONSTANTS(String logPathPrefix) {
		return new TalonFXSteerConstants(
			IDs.TalonFXIDs.BACK_RIGHT_STEER_MOTOR,
			true,
			IDs.CANCodersIDs.BACK_RIGHT_ENCODER.ID(),
			MOTOR_CONFIG,
			ENABLE_FOC,
			SYSID_CONFIG,
			logPathPrefix + ModuleUtils.ModulePosition.BACK_RIGHT + "/"
		);
	}

}
