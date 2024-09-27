package frc.robot.subsystems.pivot.factories;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.IDs;
import frc.robot.hardware.motor.phoenix6.TalonFXMotor;
import frc.robot.hardware.motor.phoenix6.TalonFXWrapper;
import frc.robot.hardware.request.phoenix6.Phoenix6AngleRequest;
import frc.robot.hardware.signal.phoenix.Phoenix6AngleSignal;
import frc.robot.hardware.signal.phoenix.Phoenix6DoubleSignal;
import frc.robot.hardware.signal.phoenix.Phoenix6LatencySignal;
import frc.robot.hardware.signal.phoenix.Phoenix6SignalBuilder;
import frc.robot.subsystems.pivot.PivotConstants;
import frc.robot.subsystems.pivot.PivotStuff;
import frc.utils.AngleUnit;
import frc.utils.alerts.Alert;

public class RealPivotConstants {

	private static final int APPLY_CONFIG_RETRIES = 5;

	private static SysIdRoutine.Config generateSysidConfig() {
		return new SysIdRoutine.Config(
				Units.Volts.of(1.0).per(Units.Seconds.of(1.0)),
				Units.Volts.of(7.0),
				Units.Seconds.of(10.0),
				(state) -> SignalLogger.writeString("state", state.toString())
		);
	}

	private static TalonFXConfiguration generateMotorConfig() {
		TalonFXConfiguration configuration = new TalonFXConfiguration();

		configuration.Slot0.kP = 198;
		configuration.Slot0.kD = 3.6;
		configuration.Slot0.kV = 18.57;
		configuration.Slot0.kS = 0.26;
		configuration.Slot0.kA = 0.7702;
		configuration.Feedback.SensorToMechanismRatio = PivotConstants.GEAR_RATIO;

		configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
		configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = PivotConstants.FORWARD_ANGLE_LIMIT.getRotations();
		configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
		configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = PivotConstants.BACKWARD_ANGLE_LIMIT.getRotations();

		configuration.CurrentLimits.StatorCurrentLimitEnable = true;
		configuration.CurrentLimits.StatorCurrentLimit = 40;

		configuration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

		return configuration;
	}

	protected static PivotStuff generatePivotStuff(String logPath) {
		Phoenix6AngleRequest positionRequest = new Phoenix6AngleRequest(new PositionVoltage(0).withEnableFOC(true));

		TalonFXWrapper motor = new TalonFXWrapper(IDs.TalonFXs.PIVOT_MOTOR_ID);
		if (!motor.applyConfiguration(generateMotorConfig(), APPLY_CONFIG_RETRIES).isOK()) {
			new Alert(Alert.AlertType.WARNING, logPath + "ConfigurationFail").report();
		}

		Phoenix6AngleSignal velocitySignal = Phoenix6SignalBuilder
			.generatePhoenix6Signal(motor.getVelocity(), GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.ROTATIONS);
		Phoenix6LatencySignal positionSignal = Phoenix6SignalBuilder
			.generatePhoenix6Signal(motor.getPosition(), velocitySignal, GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.ROTATIONS);
		Phoenix6DoubleSignal currentSignal = Phoenix6SignalBuilder
			.generatePhoenix6Signal(motor.getStatorCurrent(), GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ);
		Phoenix6DoubleSignal voltageSignal = Phoenix6SignalBuilder
			.generatePhoenix6Signal(motor.getMotorVoltage(), GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ);

		TalonFXMotor pivot = new TalonFXMotor(logPath, motor, generateSysidConfig());
		return new PivotStuff(pivot, positionRequest, positionSignal, velocitySignal, currentSignal, voltageSignal);
	}

}
