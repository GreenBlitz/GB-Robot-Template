package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.IDs;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotType;
import frc.robot.hardware.mechanisms.wpilib.FlywheelSimulation;
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

public class KrakenX60FlyWheelBuilder {

	public static FlyWheel build(String logPath, Phoenix6DeviceID motorID) {
		TalonFXFollowerConfig followerConfig = buildFollowerConfig();
		FlywheelSimulation simulationMotor = new FlywheelSimulation(
			new FlywheelSim(
				LinearSystemId.createFlywheelSystem(
					DCMotor.getKrakenX60Foc(followerConfig.followerIDs.length + 1),
					Constants.MOMENT_OF_INERTIA,
					Constants.ROTOR_TO_SENSOR_RATIO_MASTER * Constants.SENSOR_TO_MECHANISM_RATIO_MASTER
				),
				DCMotor.getKrakenX60Foc(followerConfig.followerIDs.length + 1)
			)
		);

		TalonFXMotor motor = new TalonFXMotor(logPath, motorID, followerConfig, new SysIdRoutine.Config(), simulationMotor);

		Phoenix6AngleSignal velocitySignal = Phoenix6SignalBuilder
			.build(motor.getDevice().getVelocity(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.ROTATIONS, motorID.busChain());
		Phoenix6DoubleSignal voltageSignal = Phoenix6SignalBuilder
			.build(motor.getDevice().getMotorVoltage(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, motorID.busChain());
		Phoenix6DoubleSignal currentSignal = Phoenix6SignalBuilder
			.build(motor.getDevice().getMotorStallCurrent(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, motorID.busChain());

		Phoenix6Request<Rotation2d> velocityRequest = Phoenix6RequestBuilder.build(new VelocityVoltage(0), 0, true);
		Phoenix6Request<Double> voltageRequest = Phoenix6RequestBuilder.build(new VoltageOut(0), true);

		motor.applyConfiguration(buildConfig());

		return new FlyWheel(logPath, velocityRequest, voltageRequest, velocitySignal, voltageSignal, currentSignal, motor);
	}

	public static TalonFXConfiguration buildConfig() {
		TalonFXConfiguration configuration = new TalonFXConfiguration();
		configuration.CurrentLimits.StatorCurrentLimit = 40;
		configuration.CurrentLimits.StatorCurrentLimitEnable = true;
		configuration.Feedback.SensorToMechanismRatio = Constants.SENSOR_TO_MECHANISM_RATIO_MASTER;
		configuration.Feedback.RotorToSensorRatio = Constants.ROTOR_TO_SENSOR_RATIO_MASTER;
		if (Robot.ROBOT_TYPE.equals(RobotType.REAL)) {
			configuration.Slot0.kP = Constants.KP;
			configuration.Slot0.kI = Constants.KI;
			configuration.Slot0.kD = Constants.KD;
			configuration.Slot0.kV = Constants.KV;
			configuration.Slot0.kA = Constants.KA;
			configuration.Slot0.kS = Constants.KS;
		} else {
			configuration.Slot0.kP = Constants.KP_SIM;
			configuration.Slot0.kI = Constants.KI_SIM;
			configuration.Slot0.kD = Constants.KD_SIM;
			configuration.Slot0.kV = Constants.KV_SIM;
			configuration.Slot0.kA = Constants.KA_SIM;
			configuration.Slot0.kS = Constants.KS_SIM;
		}
		return configuration;
	}

	public static TalonFXFollowerConfig buildFollowerConfig() {
		TalonFXFollowerConfig followerConfig = new TalonFXFollowerConfig();
		followerConfig.motorConfig.Feedback.SensorToMechanismRatio = Constants.SENSOR_TO_MECHANISM_RATIO_FOLLOWER;
		followerConfig.motorConfig.CurrentLimits.StatorCurrentLimit = 40;
		followerConfig.motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		followerConfig.motorConfig.Feedback.RotorToSensorRatio = Constants.ROTOR_TO_SENSOR_RATIO_FOLLOWER;
		followerConfig.followerIDs = new TalonFXFollowerConfig.TalonFXFollowerID[] {
			new TalonFXFollowerConfig.TalonFXFollowerID("flyWheelFollower", IDs.TalonFXIDs.FLYWHEEL_FOLLOWER, false)};
		return followerConfig;
	}

}
