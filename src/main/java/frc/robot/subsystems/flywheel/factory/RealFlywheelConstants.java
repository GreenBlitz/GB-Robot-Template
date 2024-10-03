package frc.robot.subsystems.flywheel.factory;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.IDs;
import frc.robot.hardware.motor.phoenix6.TalonFXMotor;
import frc.robot.hardware.motor.phoenix6.TalonFXWrapper;
import frc.robot.hardware.request.phoenix6.Phoenix6AngleRequest;
import frc.robot.hardware.request.phoenix6.Phoenix6DoubleRequest;
import frc.robot.hardware.signal.InputSignal;
import frc.robot.hardware.signal.phoenix.Phoenix6AngleSignal;
import frc.robot.hardware.signal.phoenix.Phoenix6DoubleSignal;
import frc.robot.hardware.signal.phoenix.Phoenix6SignalBuilder;
import frc.robot.subsystems.flywheel.FlyWheelConstants;
import frc.robot.subsystems.flywheel.FlywheelStuff;
import frc.utils.AngleUnit;
import frc.utils.alerts.Alert;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

public class RealFlywheelConstants {

	private static final int APPLY_CONFIG_RETRIES = 5;

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

		configuration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

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


	public static FlywheelStuff generateFlywheelStuff(String logPath) {
		String rightLogPath = logPath + "right/";
		String leftLogPath = logPath + "left/";

		Phoenix6AngleRequest rightVelocityRequest = new Phoenix6AngleRequest(new VelocityVoltage(0).withEnableFOC(true));
		Phoenix6AngleRequest leftVelocityRequest = new Phoenix6AngleRequest(new VelocityVoltage(0).withEnableFOC(true));

		TalonFXWrapper rightMotor = new TalonFXWrapper(IDs.TalonFXIDs.RIGHT_FLYWHEEL);
		if (!rightMotor.applyConfiguration(generateMotorConfig(), APPLY_CONFIG_RETRIES).isOK()) {
			new Alert(Alert.AlertType.ERROR, rightLogPath + "ConfigurationFail").report();
		}

		Phoenix6AngleSignal rightVelocitySignal = Phoenix6SignalBuilder
			.generatePhoenix6Signal(rightMotor.getVelocity(), GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.ROTATIONS);
		Phoenix6DoubleSignal rightCurrentSignal = Phoenix6SignalBuilder
			.generatePhoenix6Signal(rightMotor.getStatorCurrent(), GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ);
		Phoenix6DoubleSignal rightVoltageSignal = Phoenix6SignalBuilder
			.generatePhoenix6Signal(rightMotor.getMotorVoltage(), GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ);

		TalonFXMotor rightFlywheel = new TalonFXMotor(rightLogPath, rightMotor, generateSysidConfig());

		TalonFXWrapper leftMotor = new TalonFXWrapper(IDs.TalonFXIDs.LEFT_FLYWHEEL);
		if (!leftMotor.applyConfiguration(generateMotorConfig(), APPLY_CONFIG_RETRIES).isOK()) {
			new Alert(Alert.AlertType.ERROR, leftLogPath + "ConfigurationFail").report();
		}

		Phoenix6AngleSignal leftVelocitySignal = Phoenix6SignalBuilder
			.generatePhoenix6Signal(leftMotor.getVelocity(), GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.ROTATIONS);
		Phoenix6DoubleSignal leftCurrentSignal = Phoenix6SignalBuilder
			.generatePhoenix6Signal(leftMotor.getStatorCurrent(), GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ);
		Phoenix6DoubleSignal leftVoltageSignal = Phoenix6SignalBuilder
			.generatePhoenix6Signal(leftMotor.getMotorVoltage(), GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ);

		TalonFXMotor leftFlywheel = new TalonFXMotor(leftLogPath, leftMotor, generateSysidConfig());

		Phoenix6DoubleRequest leftVoltageRequest = new Phoenix6DoubleRequest(new VoltageOut(0).withEnableFOC(true));
		Phoenix6DoubleRequest rightVoltageRequest = new Phoenix6DoubleRequest(new VoltageOut(0).withEnableFOC(true));

		return new FlywheelStuff(
			logPath,
			rightFlywheel,
			leftFlywheel,
			rightVelocityRequest,
			leftVelocityRequest,
			rightVoltageRequest,
			leftVoltageRequest,
			rightVelocitySignal,
			leftVelocitySignal,
			new InputSignal[] {rightCurrentSignal, rightVoltageSignal},
			new InputSignal[] {leftCurrentSignal, leftVoltageSignal}
		);
	}

}
