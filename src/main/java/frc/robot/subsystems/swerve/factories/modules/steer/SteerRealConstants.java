package frc.robot.subsystems.swerve.factories.modules.steer;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
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
import frc.robot.subsystems.swerve.module.stuffs.SteerStuff;
import frc.utils.AngleUnit;
import frc.utils.alerts.Alert;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

class SteerRealConstants {

	private static final int APPLY_CONFIG_RETRIES = 5;

	private static SysIdRoutine.Config generateSysidConfig() {
		return new SysIdRoutine.Config(
			Volts.of(0.5).per(Seconds.of(1)),
			Volts.of(1),
			null,
			state -> SignalLogger.writeString("state", state.toString())
		);
	}

	private static TalonFXConfiguration generateMotorConfig(boolean inverted) {
		TalonFXConfiguration steerConfig = new TalonFXConfiguration();

		steerConfig.MotorOutput.Inverted = inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

		steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		steerConfig.CurrentLimits.StatorCurrentLimit = 30;
		steerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

		steerConfig.Feedback.RotorToSensorRatio = 150.0 / 7.0;
		steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;

		steerConfig.Slot0.kS = 0.19648;
		steerConfig.Slot0.kV = 2.5763;
		steerConfig.Slot0.kA = 0.50361;
		steerConfig.Slot0.kP = 88;
		steerConfig.Slot0.kI = 0;
		steerConfig.Slot0.kD = 1.5;
		steerConfig.ClosedLoopGeneral.ContinuousWrap = true;

		return steerConfig;
	}

	protected static SteerStuff generateSteerStuff(String logPath, Phoenix6DeviceID deviceID, Phoenix6DeviceID encoderID, boolean inverted) {
		Phoenix6AngleRequest positionRequest = new Phoenix6AngleRequest(new PositionVoltage(0).withEnableFOC(true));
		Phoenix6DoubleRequest voltageRequest = new Phoenix6DoubleRequest(new VoltageOut(0).withEnableFOC(true));

		TalonFXWrapper motor = new TalonFXWrapper(deviceID);
		TalonFXConfiguration configuration = generateMotorConfig(inverted);
		configuration.Feedback.FeedbackRemoteSensorID = encoderID.ID();
		if (!motor.applyConfiguration(configuration, APPLY_CONFIG_RETRIES).isOK()) {
			new Alert(Alert.AlertType.ERROR, logPath + "ConfigurationFailAt").report();
		}

		TalonFXMotor steer = new TalonFXMotor(logPath, motor, generateSysidConfig());
		Phoenix6DoubleSignal voltageSignal = Phoenix6SignalBuilder
			.generatePhoenix6Signal(motor.getMotorVoltage(), GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ);
		Phoenix6DoubleSignal currentSignal = Phoenix6SignalBuilder
			.generatePhoenix6Signal(motor.getStatorCurrent(), GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ);
		Phoenix6AngleSignal velocitySignal = Phoenix6SignalBuilder
			.generatePhoenix6Signal(motor.getVelocity(), GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.ROTATIONS);
		Phoenix6LatencySignal positionSignal = Phoenix6SignalBuilder
			.generatePhoenix6Signal(motor.getPosition(), velocitySignal, GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.ROTATIONS);

		return new SteerStuff(steer, positionRequest, voltageRequest, positionSignal, velocitySignal, currentSignal, voltageSignal);
	}

}
