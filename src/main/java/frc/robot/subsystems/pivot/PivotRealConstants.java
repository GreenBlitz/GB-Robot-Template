package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hardware.motor.phoenix6.TalonFXMotor;
import frc.robot.hardware.motor.phoenix6.TalonFXWrapper;

public class PivotRealConstants {


	protected static final SysIdRoutine.Config SYS_ID_CONFIG = new SysIdRoutine.Config(
		Units.Volts.of(1.0).per(Units.Seconds.of(1.0)),
		Units.Volts.of(7.0),
		Units.Seconds.of(10.0)
	);

	protected static final Rotation2d BACKWARD_ANGLE_LIMIT = Rotation2d.fromDegrees(16.5);

	protected static final TalonFXConfiguration TALON_FX_CONFIGURATION = new TalonFXConfiguration();
	static {
		Slot0Configs SLOT_0_CONFIGS = new Slot0Configs();
		SLOT_0_CONFIGS.kP = 0.5;

		FeedbackConfigs FEED_BACK_CONFIGS = new FeedbackConfigs();
		FEED_BACK_CONFIGS.RotorToSensorRatio = 15 * (72 / 14.0) * 2;


		TALON_FX_CONFIGURATION.withSlot0(SLOT_0_CONFIGS);
		TALON_FX_CONFIGURATION.withFeedback(FEED_BACK_CONFIGS);

	}

	protected static TalonFXMotor getTalonFXMotor(String logPath, TalonFXWrapper talonFXWrapper){
		return new TalonFXMotor(
				logPath,
				talonFXWrapper,
				SYS_ID_CONFIG
		);
	}
}
