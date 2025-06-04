package frc.robot.subsystems.swerve.module.factory;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.angleencoder.CANCoderEncoder;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.hardware.phoenix6.request.Phoenix6RequestBuilder;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.robot.subsystems.swerve.module.Module;
import frc.robot.subsystems.swerve.module.ModuleRequests;
import frc.robot.subsystems.swerve.module.ModuleSignals;
import frc.utils.AngleUnit;
import frc.utils.calibration.sysid.SysIdCalibrator;

public class ModuleFactory {

	private static final boolean enableFOC = true;

	private static boolean ARE_MOTORS_CTRE = true;

	private static final double DEFAULT_ARBITRARY_FEED_FORWARD = 0;
	private static double MAX_DRIVE_VELOCITY_MPS = 10;
	private static double WHEEL_DIAMETER_METERS = 0.05;
	private static boolean IS_DRIVE_INVERTED = false;
	private static boolean IS_STEER_INVERTED = false;


	public static Module build(String logPath, ModuleIDs ids) {
		String finalLogPath = ModuleConstants.LOG_PATH_PREFIX + logPath;

		SysIdCalibrator.SysIdConfigInfo configInfo = new SysIdCalibrator.SysIdConfigInfo(new SysIdRoutine.Config(), ARE_MOTORS_CTRE);

		TalonFXMotor drive = generateDrive(finalLogPath, ids.driveMotorId());
		drive.applyConfiguration(generateDriveConfig());

		TalonFXMotor steer = generateSteer(finalLogPath, ids.steerMotorId());
		steer.applyConfiguration(generateSteerConfig());

		CANCoderEncoder encoder = generateEncoder(finalLogPath, ids.encoderId());
		encoder.getDevice().getConfigurator().apply(generateEncoderConfig());

		ModuleRequests requests = generateRequests();
		ModuleSignals signals = generateSignals(drive, steer, encoder);

		return new Module(finalLogPath, drive, steer, encoder, requests, signals, configInfo, MAX_DRIVE_VELOCITY_MPS, WHEEL_DIAMETER_METERS);
	}

	private static TalonFXMotor generateDrive(String logPath, int driveID) {
		return new TalonFXMotor(logPath + "/Drive", new Phoenix6DeviceID(driveID, BusChain.ROBORIO), new SysIdRoutine.Config());
	}

	private static TalonFXMotor generateSteer(String logPath, int steerID) {
		return new TalonFXMotor(logPath + "/Steer", new Phoenix6DeviceID(steerID, BusChain.ROBORIO), new SysIdRoutine.Config());
	}

	private static CANCoderEncoder generateEncoder(String logPath, int encoderID) {
		return new CANCoderEncoder(logPath + "/Encoder", new CANcoder(encoderID, BusChain.ROBORIO.getChainName()));
	}

	private static ModuleRequests generateRequests() {
		IRequest<Rotation2d> driveVelocityRequest = Phoenix6RequestBuilder
			.build(new VelocityVoltage(0), DEFAULT_ARBITRARY_FEED_FORWARD, enableFOC);
		IRequest<Double> driveVoltageRequest = Phoenix6RequestBuilder.build(new VoltageOut(0), enableFOC);
		IRequest<Rotation2d> steerAngleRequest = Phoenix6RequestBuilder.build(new PositionVoltage(0), DEFAULT_ARBITRARY_FEED_FORWARD, enableFOC);

		return new ModuleRequests(driveVelocityRequest, driveVoltageRequest, steerAngleRequest);
	}

	private static ModuleSignals generateSignals(TalonFXMotor drive, TalonFXMotor steer, CANCoderEncoder encoder) {
		InputSignal<Rotation2d> driveVelocitySignal = Phoenix6SignalBuilder
			.build(drive.getDevice().getVelocity(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.ROTATIONS, BusChain.ROBORIO);
		InputSignal<Double> driveVoltageSignal = Phoenix6SignalBuilder
			.build(drive.getDevice().getMotorVoltage(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, BusChain.ROBORIO);
		InputSignal<Double> driveCurrentSignal = Phoenix6SignalBuilder
			.build(drive.getDevice().getMotorStallCurrent(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, BusChain.ROBORIO);
		InputSignal<Rotation2d> steerAngleSignal = Phoenix6SignalBuilder
			.build(steer.getDevice().getPosition(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.ROTATIONS, BusChain.ROBORIO);
		InputSignal<Double> steerVoltageSignal = Phoenix6SignalBuilder
			.build(steer.getDevice().getMotorVoltage(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, BusChain.ROBORIO);
		InputSignal<Rotation2d> encoderAngleSignal = Phoenix6SignalBuilder
			.build(encoder.getDevice().getPosition(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.ROTATIONS, BusChain.ROBORIO);


		return new ModuleSignals(
			driveVelocitySignal,
			driveVoltageSignal,
			driveCurrentSignal,
			steerAngleSignal,
			steerVoltageSignal,
			encoderAngleSignal
		);
	}

	private static TalonFXConfiguration generateDriveConfig() {
		TalonFXConfiguration config = new TalonFXConfiguration();

		switch (Robot.ROBOT_TYPE) {
			case REAL -> {
				config.Slot0.kP = 1;
				config.Slot0.kI = 0;
				config.Slot0.kD = 0;

				config.Slot0.kG = 0;
				config.Slot0.kS = 0;
				config.Slot0.kV = 0;
				config.Slot0.kA = 0;
			}
			case SIMULATION -> {
				config.Slot0.kP = 1;
				config.Slot0.kI = 0;
				config.Slot0.kD = 0;

				config.Slot0.kG = 0;
				config.Slot0.kS = 0;
				config.Slot0.kV = 0;
				config.Slot0.kA = 0;
			}
		}

		config.MotorOutput.Inverted = IS_DRIVE_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

		config.Feedback.RotorToSensorRatio = 1 / 1;
		config.Feedback.SensorToMechanismRatio = 1 / 1;

		config.CurrentLimits.StatorCurrentLimit = 40;
		config.CurrentLimits.StatorCurrentLimitEnable = true;
		config.CurrentLimits.SupplyCurrentLimit = 44;
		config.CurrentLimits.SupplyCurrentLimitEnable = true;

		return config;
	}

	private static TalonFXConfiguration generateSteerConfig() {
		TalonFXConfiguration config = new TalonFXConfiguration();

		switch (Robot.ROBOT_TYPE) {
			case REAL -> {
				config.Slot0.kP = 1;
				config.Slot0.kI = 0;
				config.Slot0.kD = 0;

				config.Slot0.kG = 0;
				config.Slot0.kS = 0;
				config.Slot0.kV = 0;
				config.Slot0.kA = 0;
			}
			case SIMULATION -> {
				config.Slot0.kP = 1;
				config.Slot0.kI = 0;
				config.Slot0.kD = 0;

				config.Slot0.kG = 0;
				config.Slot0.kS = 0;
				config.Slot0.kV = 0;
				config.Slot0.kA = 0;
			}
		}

		config.MotorOutput.Inverted = IS_STEER_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

		config.Feedback.RotorToSensorRatio = 1 / 1;

		config.CurrentLimits.StatorCurrentLimit = 40;
		config.CurrentLimits.StatorCurrentLimitEnable = true;
		config.CurrentLimits.SupplyCurrentLimit = 44;
		config.CurrentLimits.SupplyCurrentLimitEnable = true;
		return config;
	}

	private static CANcoderConfiguration generateEncoderConfig() {
		CANcoderConfiguration config = new CANcoderConfiguration();

		config.MagnetSensor.MagnetOffset = 0;
		config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
		config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;

		return config;
	}

}
