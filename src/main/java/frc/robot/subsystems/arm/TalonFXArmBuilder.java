package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.hardware.interfaces.IDynamicMotionMagicRequest;
import frc.robot.hardware.mechanisms.wpilib.SingleJointedArmSimulation;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.motors.TalonFXFollowerConfig;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.hardware.phoenix6.request.Phoenix6FeedForwardRequest;
import frc.robot.hardware.phoenix6.request.Phoenix6Request;
import frc.robot.hardware.phoenix6.request.Phoenix6RequestBuilder;
import frc.robot.hardware.phoenix6.signal.Phoenix6AngleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.utils.AngleUnit;

public class TalonFXArmBuilder {

	public static DynamicMotionMagicArm buildDynamicMotionMagicArm(
		String logPath,
		Phoenix6DeviceID deviceID,
		boolean isInverted,
		boolean isContinuesWrap,
		TalonFXFollowerConfig talonFXFollowerConfig,
		SysIdRoutine.Config sysIdRoutineConfig,
		FeedbackConfigs feedbackConfigs,
		Slot0Configs realSlotsConfig,
		Slot0Configs simulationSlotsConfig,
		double currentLimit,
		double signalsFrequency,
		double arbitraryFeedForward,
		Rotation2d forwardSoftwareLimit,
		Rotation2d reverseSoftwareLimit,
		ArmSimulationConstants simulationConstants,
		Rotation2d defaultMaxAccelerationRotation2dPerSecondSquare,
		Rotation2d defaultMaxVelocityRotation2dPerSecond
	) {
		TalonFXMotor motor = new TalonFXMotor(
			logPath,
			deviceID,
			talonFXFollowerConfig,
			sysIdRoutineConfig,
			buildSimulation(
				simulationConstants,
				talonFXFollowerConfig,
				feedbackConfigs.RotorToSensorRatio * feedbackConfigs.SensorToMechanismRatio
			)
		);

		ArmSignals signals = buildSignals(motor, signalsFrequency, deviceID.busChain());

		Phoenix6Request<Double> voltageRequest = buildVoltageRequest();

		IDynamicMotionMagicRequest positionRequest = Phoenix6RequestBuilder.build(
			new DynamicMotionMagicVoltage(
				signals.position().getLatestValue().getRotations(),
				defaultMaxVelocityRotation2dPerSecond.getRotations(),
				defaultMaxAccelerationRotation2dPerSecondSquare.getRotations()
			),
			arbitraryFeedForward,
			true
		);
		TalonFXConfiguration configuration = buildConfiguration(
			feedbackConfigs,
			simulationSlotsConfig,
			realSlotsConfig,
			forwardSoftwareLimit,
			reverseSoftwareLimit,
			isInverted,
			isContinuesWrap,
			currentLimit
		);
		addMotionMagicConfig(configuration, defaultMaxVelocityRotation2dPerSecond, defaultMaxAccelerationRotation2dPerSecondSquare);
		motor.applyConfiguration(configuration);

		return new DynamicMotionMagicArm(
			logPath,
			motor,
			signals,
			voltageRequest,
			positionRequest,
			defaultMaxAccelerationRotation2dPerSecondSquare,
			defaultMaxVelocityRotation2dPerSecond,
			configuration.Slot0.kG
		);
	}

	public static Arm buildMotionMagicArm(
		String logPath,
		Phoenix6DeviceID deviceID,
		boolean isInverted,
		boolean isContinuesWrap,
		TalonFXFollowerConfig talonFXFollowerConfig,
		SysIdRoutine.Config sysIdRoutineConfig,
		FeedbackConfigs feedbackConfigs,
		Slot0Configs realSlotsConfig,
		Slot0Configs simulationSlotsConfig,
		double currentLimit,
		double signalsFrequency,
		double arbitraryFeedForward,
		Rotation2d forwardSoftwareLimit,
		Rotation2d reverseSoftwareLimit,
		ArmSimulationConstants simulationConstants,
		Rotation2d defaultMaxAccelerationRotation2dPerSecondSquare,
		Rotation2d defaultMaxVelocityRotation2dPerSecond
	) {
		TalonFXMotor motor = new TalonFXMotor(
			logPath,
			deviceID,
			talonFXFollowerConfig,
			sysIdRoutineConfig,
			buildSimulation(
				simulationConstants,
				talonFXFollowerConfig,
				feedbackConfigs.RotorToSensorRatio * feedbackConfigs.SensorToMechanismRatio
			)
		);

		ArmSignals signals = buildSignals(motor, signalsFrequency, deviceID.busChain());

		Phoenix6Request<Double> voltageRequest = buildVoltageRequest();

		Phoenix6FeedForwardRequest positionRequest = Phoenix6RequestBuilder
			.build(new MotionMagicVoltage(signals.position().getLatestValue().getRotations()), arbitraryFeedForward, true);
		TalonFXConfiguration configuration = (buildConfiguration(
			feedbackConfigs,
			simulationSlotsConfig,
			realSlotsConfig,
			forwardSoftwareLimit,
			reverseSoftwareLimit,
			isInverted,
			isContinuesWrap,
			currentLimit
		));
		addMotionMagicConfig(configuration, defaultMaxVelocityRotation2dPerSecond, defaultMaxAccelerationRotation2dPerSecondSquare);
		motor.applyConfiguration(configuration);

		return new Arm(logPath, motor, signals, voltageRequest, positionRequest, configuration.Slot0.kG);
	}

	public static Arm buildArm(
		String logPath,
		Phoenix6DeviceID deviceID,
		boolean isInverted,
		boolean isContinuesWrap,
		TalonFXFollowerConfig talonFXFollowerConfig,
		SysIdRoutine.Config sysIdRoutineConfig,
		FeedbackConfigs feedbackConfigs,
		Slot0Configs realSlotsConfig,
		Slot0Configs simulationSlotsConfig,
		double currentLimit,
		double signalsFrequency,
		double arbitraryFeedForward,
		Rotation2d forwardSoftwareLimit,
		Rotation2d reverseSoftwareLimit,
		ArmSimulationConstants simulationConstants
	) {
		TalonFXMotor motor = new TalonFXMotor(
			logPath,
			deviceID,
			talonFXFollowerConfig,
			sysIdRoutineConfig,
			buildSimulation(
				simulationConstants,
				talonFXFollowerConfig,
				feedbackConfigs.RotorToSensorRatio * feedbackConfigs.SensorToMechanismRatio
			)
		);

		ArmSignals signals = buildSignals(motor, signalsFrequency, deviceID.busChain());

		Phoenix6Request<Double> voltageRequest = buildVoltageRequest();

		Phoenix6FeedForwardRequest positionRequest = Phoenix6RequestBuilder
			.build(new PositionVoltage(signals.position().getLatestValue().getRotations()), arbitraryFeedForward, true);

		TalonFXConfiguration configuration = buildConfiguration(
			feedbackConfigs,
			simulationSlotsConfig,
			realSlotsConfig,
			forwardSoftwareLimit,
			reverseSoftwareLimit,
			isInverted,
			isContinuesWrap,
			currentLimit
		);
		motor.applyConfiguration(configuration);
		return new Arm(logPath, motor, signals, voltageRequest, positionRequest, configuration.Slot0.kG);
	}

	private static TalonFXConfiguration buildConfiguration(
		FeedbackConfigs feedbackConfigs,
		Slot0Configs simulationConfigSlots,
		Slot0Configs realConfigSlots,
		Rotation2d forwardSoftwareLimit,
		Rotation2d reverseSoftwareLimit,
		boolean isInverted,
		boolean isContinuesWrap,
		double currentLimit
	) {
		TalonFXConfiguration config = new TalonFXConfiguration();

		switch (Robot.ROBOT_TYPE) {
			case REAL, REPLAY -> {
				config.Slot0 = realConfigSlots;
			}
			case SIMULATION -> {
				config.Slot0 = simulationConfigSlots;
			}
		}
		config.Feedback = feedbackConfigs;

		config.ClosedLoopGeneral.ContinuousWrap = isContinuesWrap;

		config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseSoftwareLimit.getRotations();
		config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardSoftwareLimit.getRotations();
		config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
		config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

		config.CurrentLimits.StatorCurrentLimitEnable = true;
		config.CurrentLimits.StatorCurrentLimit = currentLimit;

		config.MotorOutput.Inverted = isInverted ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
		config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

		return config;
	}

	private static void addMotionMagicConfig(TalonFXConfiguration config, Rotation2d maxVelocity, Rotation2d maxAcceleration) {
		config.MotionMagic.MotionMagicAcceleration = maxAcceleration.getRotations();
		config.MotionMagic.MotionMagicCruiseVelocity = maxVelocity.getRotations();
	}

	private static SingleJointedArmSimulation buildSimulation(
		ArmSimulationConstants simulationConstants,
		TalonFXFollowerConfig followerConfig,
		double gearing
	) {
		return new SingleJointedArmSimulation(
			new SingleJointedArmSim(
				LinearSystemId.createDCMotorSystem(
					DCMotor.getKrakenX60Foc(followerConfig.followerIDs.length + 1),
					simulationConstants.momentOfInertiaKgMeterSquared(),
					gearing
				),
				DCMotor.getKrakenX60Foc(followerConfig.followerIDs.length + 1),
				gearing,
				simulationConstants.armLengthMeters(),
				simulationConstants.minPosition().getRadians(),
				simulationConstants.maxPosition().getRadians(),
				false,
				simulationConstants.startingPosition().getRadians()
			),
			gearing
		);
	}

	private static Phoenix6Request<Double> buildVoltageRequest() {
		return Phoenix6RequestBuilder.build(new VoltageOut(0), true);
	}

	private static ArmSignals buildSignals(TalonFXMotor motor, double signalFrequency, BusChain busChain) {
		Phoenix6AngleSignal velocity = Phoenix6SignalBuilder
			.build(motor.getDevice().getVelocity(), signalFrequency, AngleUnit.ROTATIONS, busChain);
		return new ArmSignals(
			Phoenix6SignalBuilder.build(motor.getDevice().getMotorVoltage(), signalFrequency, busChain),
			Phoenix6SignalBuilder.build(motor.getDevice().getStatorCurrent(), signalFrequency, busChain),
			velocity,
			Phoenix6SignalBuilder.build(motor.getDevice().getPosition(), velocity, signalFrequency, AngleUnit.ROTATIONS, busChain)
		);
	}

}
