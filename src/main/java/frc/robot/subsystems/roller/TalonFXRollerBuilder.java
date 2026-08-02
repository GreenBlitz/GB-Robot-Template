package frc.robot.subsystems.roller;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.hardware.mechanisms.wpilib.SimpleMotorSimulation;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.motors.TalonFXFollowerConfig;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.hardware.phoenix6.request.Phoenix6FeedForwardRequest;
import frc.robot.hardware.phoenix6.request.Phoenix6Request;
import frc.robot.hardware.phoenix6.request.Phoenix6RequestBuilder;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.utils.AngleUnit;

public class TalonFXRollerBuilder {

	public static VelocityRoller buildVelocityRoller(
		String logPath,
		Phoenix6DeviceID deviceID,
		Slot0Configs realVelocityControlConfig,
		Slot0Configs simulationVelocityControlConfig,
		int currentLimit,
		FeedbackConfigs feedbackConfigs,
		double momentOfInertia,
		boolean isInverted
	) {
		SimpleMotorSimulation rollerSimulation = new SimpleMotorSimulation(
			new DCMotorSim(
				LinearSystemId.createDCMotorSystem(
					DCMotor.getKrakenX60(1),
					momentOfInertia,
					feedbackConfigs.SensorToMechanismRatio * feedbackConfigs.RotorToSensorRatio
				),
				DCMotor.getKrakenX60(1)
			)
		);
		TalonFXMotor roller = new TalonFXMotor(logPath, deviceID, new TalonFXFollowerConfig(), new SysIdRoutine.Config(), rollerSimulation);

		roller.applyConfiguration(
			buildConfiguration(
				isInverted,
				feedbackConfigs,
				currentLimit,
				Robot.ROBOT_TYPE.isSimulation() ? simulationVelocityControlConfig : realVelocityControlConfig
			)
		);

		InputSignal<Double> voltageSignal = Phoenix6SignalBuilder
			.build(roller.getDevice().getMotorVoltage(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, deviceID.busChain());
		InputSignal<Double> currentSignal = Phoenix6SignalBuilder
			.build(roller.getDevice().getStatorCurrent(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, deviceID.busChain());
		InputSignal<Rotation2d> positionSignal = Phoenix6SignalBuilder.build(
			roller.getDevice().getPosition(),
			roller.getDevice().getVelocity(),
			RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
			AngleUnit.ROTATIONS,
			deviceID.busChain()
		);
		InputSignal<Rotation2d> velocitySignal = Phoenix6SignalBuilder
			.build(roller.getDevice().getVelocity(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.ROTATIONS, deviceID.busChain());

		Phoenix6Request<Double> voltageRequest = Phoenix6RequestBuilder.build(new VoltageOut(0), true);
		Phoenix6FeedForwardRequest velocityVoltage = Phoenix6RequestBuilder.build(new VelocityVoltage(0), 0, true);

		return new VelocityRoller(
			logPath,
			roller,
			voltageSignal,
			currentSignal,
			positionSignal,
			velocitySignal,
			voltageRequest,
			velocityVoltage
		);
	}

	public static Roller build(
		String logPath,
		Phoenix6DeviceID id,
		boolean inverted,
		FeedbackConfigs feedbackConfigs,
		int currentLimit,
		double momentOfInertia
	) {
		SimpleMotorSimulation rollerSimulation = new SimpleMotorSimulation(
			new DCMotorSim(
				LinearSystemId.createDCMotorSystem(
					DCMotor.getKrakenX60(1),
					momentOfInertia,
					feedbackConfigs.SensorToMechanismRatio * feedbackConfigs.RotorToSensorRatio
				),
				DCMotor.getKrakenX60(1)
			)
		);
		TalonFXMotor roller = new TalonFXMotor(logPath, id, new TalonFXFollowerConfig(), new SysIdRoutine.Config(), rollerSimulation);

		roller.applyConfiguration(buildConfiguration(inverted, feedbackConfigs, currentLimit, new Slot0Configs()));

		InputSignal<Double> voltageSignal = Phoenix6SignalBuilder
			.build(roller.getDevice().getMotorVoltage(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, id.busChain());
		InputSignal<Double> currentSignal = Phoenix6SignalBuilder
			.build(roller.getDevice().getStatorCurrent(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, id.busChain());
		InputSignal<Rotation2d> positionSignal = Phoenix6SignalBuilder.build(
			roller.getDevice().getPosition(),
			roller.getDevice().getVelocity(),
			RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
			AngleUnit.ROTATIONS,
			id.busChain()
		);
		InputSignal<Rotation2d> velocitySignal = Phoenix6SignalBuilder
			.build(roller.getDevice().getVelocity(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.ROTATIONS, id.busChain());


		Phoenix6Request<Double> VoltageRequest = Phoenix6RequestBuilder.build(new VoltageOut(0), true);

		return new Roller(logPath, roller, voltageSignal, currentSignal, positionSignal, velocitySignal, VoltageRequest);
	}

	public static TalonFXConfiguration buildConfiguration(
		boolean inverted,
		FeedbackConfigs feedbackConfigs,
		int currentLimit,
		Slot0Configs velocityControlConfig
	) {
		TalonFXConfiguration configs = new TalonFXConfiguration();
		configs.Slot0 = velocityControlConfig;
		configs.CurrentLimits.SupplyCurrentLimit = currentLimit;
		configs.CurrentLimits.SupplyCurrentLimitEnable = true;
		configs.Feedback = feedbackConfigs;
		configs.MotorOutput.Inverted = inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
		return (configs);
	}

}
