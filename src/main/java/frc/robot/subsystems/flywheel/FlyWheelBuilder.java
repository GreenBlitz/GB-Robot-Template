package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
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
				LinearSystemId.createDCMotorSystem(
					DCMotor.getKrakenX60Foc(talonFXFollowerConfig.followerIDs.length),
					Constants.MOMENT_OF_INERTIA,
					Constants.ROTOR_TO_SENSOR_RATIO
				),
				DCMotor.getKrakenX60Foc(talonFXFollowerConfig.followerIDs.length)
			)
		);

		TalonFXMotor motor = new TalonFXMotor(logPath + "/master", motorID, talonFXFollowerConfig, new SysIdRoutine.Config(), simulationMotor);

		Phoenix6AngleSignal velocitySignal = Phoenix6SignalBuilder
			.build(motor.getDevice().getVelocity(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.ROTATIONS, BusChain.ROBORIO);
		Phoenix6DoubleSignal voltageSignal = Phoenix6SignalBuilder
			.build(motor.getDevice().getMotorVoltage(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, BusChain.ROBORIO);

		Phoenix6Request<Rotation2d> velocityRequest = Phoenix6RequestBuilder.build(new VelocityVoltage(0), 0, true);
		Phoenix6Request<Double> voltageRequest = Phoenix6RequestBuilder.build(new VoltageOut(0), true);

		motor.applyConfiguration(buildConfig());

		return new FlyWheel(logPath, velocityRequest, voltageRequest, velocitySignal, voltageSignal, motor);
	}

	public static TalonFXConfiguration buildConfig() {
		TalonFXConfiguration configuration = new TalonFXConfiguration();
		configuration.CurrentLimits.StatorCurrentLimit = Constants.CURRENT_LIMITS;
		configuration.CurrentLimits.StatorCurrentLimitEnable = true;
		configuration.Feedback.SensorToMechanismRatio = Constants.SENSOR_TO_MECHANISM_RATIO;
		configuration.Feedback.RotorToSensorRatio = Constants.ROTOR_TO_SENSOR_RATIO;
		configuration.Slot0.kP = Constants.KP;
		configuration.Slot0.kI = Constants.KI;
		configuration.Slot0.kD = Constants.KD;
		return configuration;
	}

}
