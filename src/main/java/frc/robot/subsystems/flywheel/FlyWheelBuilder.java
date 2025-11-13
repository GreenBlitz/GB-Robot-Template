package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotConstants;
import frc.robot.hardware.mechanisms.wpilib.SimpleMotorSimulation;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.motors.TalonFXFollowerConfig;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.hardware.phoenix6.request.Phoenix6Request;
import frc.robot.hardware.phoenix6.request.Phoenix6RequestBuilder;
import frc.robot.hardware.phoenix6.signal.Phoenix6AngleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6DoubleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.utils.AngleUnit;
import com.ctre.phoenix6.controls.VoltageOut;

public class FlyWheelBuilder {

	public static FlyWheel generate(String logPath, Phoenix6DeviceID motorID, TalonFXFollowerConfig talonFXFollowerConfig) {
		SimpleMotorSimulation simulationMotor = new SimpleMotorSimulation(
			new DCMotorSim(
				LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60Foc(2), 0.001, Constants.ROTOR_TO_SENSOR_RATIO),
				DCMotor.getKrakenX60Foc(2)
			)
		);

		TalonFXMotor masterMotor = new TalonFXMotor(
			logPath + "/master",
			motorID,
			talonFXFollowerConfig,
			new SysIdRoutine.Config(),
			simulationMotor
		);
		masterMotor.setBrake(true);
		Phoenix6DoubleSignal voltageSignalRightMotor = Phoenix6SignalBuilder
			.build(masterMotor.getDevice().getMotorVoltage(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, BusChain.ROBORIO);
		Phoenix6AngleSignal velocitySignalRightMotor = Phoenix6SignalBuilder
			.build(masterMotor.getDevice().getVelocity(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.ROTATIONS, BusChain.ROBORIO);
		Phoenix6Request<Double> voltageRequest = Phoenix6RequestBuilder.build(new VoltageOut(0), true);
		Phoenix6Request<Rotation2d> velocityRequest = Phoenix6RequestBuilder.build(new VelocityVoltage(0), 0, true);
		masterMotor.applyConfiguration(talonFXFollowerConfig.motorConfig);

		return new FlyWheel(logPath, velocityRequest, voltageRequest, velocitySignalRightMotor, voltageSignalRightMotor, masterMotor);
	}

	public static TalonFXConfiguration buildConfig() {
		TalonFXConfiguration configuration = new TalonFXConfiguration();
		configuration.CurrentLimits.SupplyCurrentLimit = Constants.CURRENT_LIMITS;
		configuration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
		configuration.Slot0.kP = Constants.KP;
		configuration.Slot0.kI = Constants.KI;
		configuration.Slot0.kD = Constants.KD;
		return configuration;
	}

}

