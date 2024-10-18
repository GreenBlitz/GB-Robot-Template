package frc.robot.subsystems.swerve.factories.modules.drive;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.GlobalConstants;
import frc.robot.hardware.motor.phoenix6.TalonFXMotor;
import frc.robot.hardware.motor.phoenix6.TalonFXWrapper;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.request.phoenix6.Phoenix6AngleRequest;
import frc.robot.hardware.request.phoenix6.Phoenix6DoubleRequest;
import frc.robot.hardware.signal.phoenix.Phoenix6AngleSignal;
import frc.robot.hardware.signal.phoenix.Phoenix6DoubleSignal;
import frc.robot.hardware.signal.phoenix.Phoenix6LatencySignal;
import frc.robot.hardware.signal.phoenix.Phoenix6SignalBuilder;
import frc.robot.subsystems.swerve.module.stuffs.DriveStuff;
import frc.utils.AngleUnit;
import frc.utils.alerts.Alert;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

class DriveRealConstants {

	private static final int APPLY_CONFIG_RETRIES = 5;

	private static final double SLIP_CURRENT = 60;

	private static SysIdRoutine.Config generateSysidConfig() {
		return new SysIdRoutine.Config(
			Volts.of(0.5).per(Seconds.of(1)),
			Volts.of(2),
			null,
			state -> SignalLogger.writeString("state", state.toString())
		);
	}

	private static TalonFXConfiguration generateMotorConfig(boolean inverted) {
		TalonFXConfiguration driveConfig = new TalonFXConfiguration();

		driveConfig.MotorOutput.Inverted = inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

		driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		driveConfig.Feedback.SensorToMechanismRatio = 8;

		driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = SLIP_CURRENT;
		driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -SLIP_CURRENT;
		driveConfig.CurrentLimits.StatorCurrentLimit = SLIP_CURRENT;
		driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;

		driveConfig.Slot0.kS = 0.21549;
		driveConfig.Slot0.kV = 0.72124;
		driveConfig.Slot0.kA = 0.11218;
		driveConfig.Slot0.kP = 1.5;
		driveConfig.Slot0.kI = 0;
		driveConfig.Slot0.kD = 0;

		return driveConfig;
	}

	protected static DriveStuff generateDriveStuff(String logPath, Phoenix6DeviceID deviceID, boolean inverted) {
		Phoenix6AngleRequest velocityRequest = new Phoenix6AngleRequest(new VelocityVoltage(0).withEnableFOC(true));
		Phoenix6DoubleRequest voltageRequest = new Phoenix6DoubleRequest(new VoltageOut(0).withEnableFOC(true));

		TalonFXWrapper motor = new TalonFXWrapper(deviceID);
		if (!motor.applyConfiguration(generateMotorConfig(inverted), APPLY_CONFIG_RETRIES).isOK()) {
			new Alert(Alert.AlertType.ERROR, logPath + "ConfigurationFailAt").report();
		}

		Phoenix6DoubleSignal voltageSignal = Phoenix6SignalBuilder
			.generatePhoenix6Signal(motor.getMotorVoltage(), GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ);
		Phoenix6DoubleSignal currentSignal = Phoenix6SignalBuilder
			.generatePhoenix6Signal(motor.getStatorCurrent(), GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ);
		Phoenix6AngleSignal velocitySignal = Phoenix6SignalBuilder
			.generatePhoenix6Signal(motor.getVelocity(), GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.ROTATIONS);
		Phoenix6LatencySignal positionSignal = Phoenix6SignalBuilder
			.generatePhoenix6Signal(motor.getPosition(), velocitySignal, GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.ROTATIONS);

		TalonFXMotor drive = new TalonFXMotor(logPath, motor, generateSysidConfig());
		return new DriveStuff(logPath, drive, velocityRequest, voltageRequest, positionSignal, velocitySignal, currentSignal, voltageSignal);
	}

}
