package frc.robot.subsystems.swerve.factories.modules.steer;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotConstants;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.mechanisms.wpilib.SimpleMotorSimulation;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.hardware.phoenix6.request.Phoenix6RequestBuilder;
import frc.robot.hardware.phoenix6.signal.Phoenix6AngleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6DoubleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6LatencySignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.robot.subsystems.swerve.module.records.SteerRequests;
import frc.robot.subsystems.swerve.module.records.SteerSignals;
import frc.utils.AngleUnit;

import static edu.wpi.first.units.Units.*;

class TalonFXSteerConstants {

	private static final double GEAR_RATIO = 150.0 / 7.0;

	private static SysIdRoutine.Config generateSysidConfig() {
		return new SysIdRoutine.Config(
			Volts.of(0.5).per(Second),
			Volts.of(1),
			null,
			state -> SignalLogger.writeString("state", state.toString())
		);
	}

	private static SimpleMotorSimulation generateMechanismSimulation() {
		double momentOfInertiaMetersSquared = 0.00001;
		return new SimpleMotorSimulation(
			new DCMotorSim(
				LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500Foc(1), momentOfInertiaMetersSquared, GEAR_RATIO),
				DCMotor.getFalcon500Foc(1)
			)
		);
	}

	private static TalonFXConfiguration generateMotorConfig(boolean inverted) {
		TalonFXConfiguration steerConfig = new TalonFXConfiguration();

		steerConfig.MotorOutput.Inverted = inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

		steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		steerConfig.CurrentLimits.StatorCurrentLimit = 30;
		steerConfig.CurrentLimits.StatorCurrentLimitEnable = true;

		steerConfig.Feedback.RotorToSensorRatio = GEAR_RATIO;
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

	protected static ControllableMotor generateSteer(String logPath, Phoenix6DeviceID deviceID, Phoenix6DeviceID encoderID, boolean inverted) {
		TalonFXConfiguration configuration = generateMotorConfig(inverted);
		configuration.Feedback.FeedbackRemoteSensorID = encoderID.id();

		TalonFXMotor steer = new TalonFXMotor(logPath, deviceID, generateSysidConfig(), generateMechanismSimulation());
		steer.applyConfiguration(configuration);
		return steer;
	}

	protected static SteerRequests generateRequests() {
		return new SteerRequests(
			Phoenix6RequestBuilder.build(new PositionVoltage(0).withEnableFOC(true)),
			Phoenix6RequestBuilder.build(new VoltageOut(0).withEnableFOC(true))
		);
	}

	protected static SteerSignals generateSignals(TalonFXMotor steer) {
		Phoenix6DoubleSignal voltageSignal = Phoenix6SignalBuilder
			.generatePhoenix6Signal(steer.getDevice().getMotorVoltage(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ);
		Phoenix6DoubleSignal currentSignal = Phoenix6SignalBuilder
			.generatePhoenix6Signal(steer.getDevice().getStatorCurrent(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ);
		Phoenix6AngleSignal velocitySignal = Phoenix6SignalBuilder
			.generatePhoenix6Signal(steer.getDevice().getVelocity(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.ROTATIONS);
		Phoenix6LatencySignal positionSignal = Phoenix6SignalBuilder.generatePhoenix6Signal(
			steer.getDevice().getPosition(),
			velocitySignal,
			RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
			AngleUnit.ROTATIONS
		);

		return new SteerSignals(positionSignal, velocitySignal, currentSignal, voltageSignal);
	}

}
