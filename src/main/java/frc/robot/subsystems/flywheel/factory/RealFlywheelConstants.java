package frc.robot.subsystems.flywheel.factory;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.GlobalConstants;
import frc.robot.hardware.motor.phoenix6.TalonFXMotor;
import frc.robot.hardware.motor.phoenix6.TalonFXWrapper;
import frc.robot.hardware.signal.phoenix.Phoenix6AngleSignal;
import frc.robot.hardware.signal.phoenix.Phoenix6SignalBuilder;
import frc.robot.subsystems.flywheel.FlyWheelConstants;
import frc.utils.AngleUnit;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

public class RealFlywheelConstants {

	private static SysIdRoutine.Config generateSysidConfig() {
		return new SysIdRoutine.Config(
				Volts.of(1).per(Seconds.of(1)),
				Volts.of(7),
				Seconds.of(10),
				(state) -> SignalLogger.writeString("state", state.toString())
		);
	}

	private static TalonFXConfiguration generateMotorConfig() {
		TalonFXConfiguration configuration = new TalonFXConfiguration();

		configuration.CurrentLimits.StatorCurrentLimit = 40;
		configuration.CurrentLimits.StatorCurrentLimitEnable = true;
		configuration.CurrentLimits.SupplyCurrentLimit = 40;
		configuration.CurrentLimits.SupplyCurrentLimitEnable = true;

		configuration.Slot0.kS = 0.26603;
		configuration.Slot0.kV = 0.11306;
		configuration.Slot0.kA = 0.027703;
		configuration.Slot0.kP = 1;

		configuration.Feedback.SensorToMechanismRatio = FlyWheelConstants.GEAR_RATIO;

		return configuration;
	}


	public static TalonFXMotor getTalonFXMotor(String logPath, TalonFXWrapper motorWrapper) {
		return new TalonFXMotor(logPath, motorWrapper, SYSID_CONFIG);
	}

	public static Phoenix6AngleSignal generateSignal(StatusSignal<Double> statusSignal) {
		return Phoenix6SignalBuilder.generatePhoenix6Signal(statusSignal, GlobalConstants.ROBORIO_CANBUS_UPDATE_FREQUENCY, AngleUnit.ROTATIONS);
	}

}
