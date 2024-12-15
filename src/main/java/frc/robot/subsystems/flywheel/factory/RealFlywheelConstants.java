package frc.robot.subsystems.flywheel.factory;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.IDs;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.hardware.mechanisms.wpilib.FlywheelSimulation;
import frc.robot.hardware.phoenix6.motor.TalonFXMotor;
import frc.robot.hardware.phoenix6.request.Phoenix6Request;
import frc.robot.hardware.phoenix6.request.Phoenix6RequestBuilder;
import frc.robot.hardware.phoenix6.signal.Phoenix6AngleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6DoubleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.robot.subsystems.flywheel.FlyWheelConstants;
import frc.robot.subsystems.flywheel.FlywheelStuff;
import frc.utils.AngleUnit;

import static edu.wpi.first.units.Units.*;

public class RealFlywheelConstants {

	private static SysIdRoutine.Config generateSysidConfig() {
		return new SysIdRoutine.Config(
			Volts.of(1).per(Second),
			Volts.of(7),
			Seconds.of(10),
			(state) -> SignalLogger.writeString("state", state.toString())
		);
	}

	private static TalonFXConfiguration generateMotorConfig() {
		TalonFXConfiguration configuration = new TalonFXConfiguration();

		configuration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

		configuration.CurrentLimits.StatorCurrentLimit = 40;
		configuration.CurrentLimits.StatorCurrentLimitEnable = true;
		configuration.CurrentLimits.SupplyCurrentLimit = 40;
		configuration.CurrentLimits.SupplyCurrentLimitEnable = true;

		configuration.Slot0.kS = 0.26603;
		configuration.Slot0.kV = 0.11306;
		configuration.Slot0.kA = 0.027703;
		configuration.Slot0.kP = 10;

		configuration.Feedback.SensorToMechanismRatio = FlyWheelConstants.GEAR_RATIO;

		return configuration;
	}

	private static TalonFXConfiguration generateSimulationMotorConfig() {
		TalonFXConfiguration configuration = new TalonFXConfiguration();

		configuration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

		configuration.CurrentLimits.StatorCurrentLimit = 40;
		configuration.CurrentLimits.StatorCurrentLimitEnable = true;
		configuration.CurrentLimits.SupplyCurrentLimit = 40;
		configuration.CurrentLimits.SupplyCurrentLimitEnable = true;

		configuration.Feedback.SensorToMechanismRatio = FlyWheelConstants.GEAR_RATIO;

		return configuration;
	}


	public static FlywheelStuff generateFlywheelStuff(String logPath) {
		String rightLogPath = logPath + "right/";
		String leftLogPath = logPath + "left/";

		Phoenix6Request<Rotation2d> rightVelocityRequest = Phoenix6RequestBuilder.build(new VelocityVoltage(0).withEnableFOC(true));
		Phoenix6Request<Rotation2d> leftVelocityRequest = Phoenix6RequestBuilder.build(new VelocityVoltage(0).withEnableFOC(true));
		Phoenix6Request<Double> rightVoltageRequest = Phoenix6RequestBuilder.build(new VoltageOut(0).withEnableFOC(true));
		Phoenix6Request<Double> leftVoltageRequest = Phoenix6RequestBuilder.build(new VoltageOut(0).withEnableFOC(true));

		TalonFXMotor rightFlywheel = new TalonFXMotor(rightLogPath, IDs.TalonFXIDs.RIGHT_FLYWHEEL, generateMotorConfig(), generateSysidConfig());

		Phoenix6AngleSignal rightVelocitySignal = Phoenix6SignalBuilder.generatePhoenix6Signal(
			rightFlywheel.getMotor().getVelocity(),
			GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
			AngleUnit.ROTATIONS
		);
		Phoenix6DoubleSignal rightCurrentSignal = Phoenix6SignalBuilder
			.generatePhoenix6Signal(rightFlywheel.getMotor().getStatorCurrent(), GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ);
		Phoenix6DoubleSignal rightVoltageSignal = Phoenix6SignalBuilder
			.generatePhoenix6Signal(rightFlywheel.getMotor().getMotorVoltage(), GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ);

		TalonFXMotor leftFlywheel = new TalonFXMotor(leftLogPath, IDs.TalonFXIDs.LEFT_FLYWHEEL, generateMotorConfig(), generateSysidConfig());

		Phoenix6AngleSignal leftVelocitySignal = Phoenix6SignalBuilder
			.generatePhoenix6Signal(leftFlywheel.getMotor().getVelocity(), GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.ROTATIONS);
		Phoenix6DoubleSignal leftCurrentSignal = Phoenix6SignalBuilder
			.generatePhoenix6Signal(leftFlywheel.getMotor().getStatorCurrent(), GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ);
		Phoenix6DoubleSignal leftVoltageSignal = Phoenix6SignalBuilder
			.generatePhoenix6Signal(leftFlywheel.getMotor().getMotorVoltage(), GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ);

		return new FlywheelStuff(
			logPath,
			rightFlywheel,
			leftFlywheel,
			rightVelocityRequest,
			leftVelocityRequest,
			rightVoltageRequest,
			leftVoltageRequest,
			rightVelocitySignal,
			leftVelocitySignal,
			new InputSignal[] {rightCurrentSignal, rightVoltageSignal},
			new InputSignal[] {leftCurrentSignal, leftVoltageSignal}
		);
	}

	public static FlywheelStuff generateSimulationFlywheelStuff(String logPath) {
		String rightLogPath = logPath + "right/";
		String leftLogPath = logPath + "left/";

		Phoenix6Request<Rotation2d> rightVelocityRequest = Phoenix6RequestBuilder.build(new VelocityVoltage(0).withEnableFOC(true));
		Phoenix6Request<Rotation2d> leftVelocityRequest = Phoenix6RequestBuilder.build(new VelocityVoltage(0).withEnableFOC(true));
		Phoenix6Request<Double> rightVoltageRequest = Phoenix6RequestBuilder.build(new VoltageOut(0).withEnableFOC(true));
		Phoenix6Request<Double> leftVoltageRequest = Phoenix6RequestBuilder.build(new VoltageOut(0).withEnableFOC(true));

		FlywheelSim rightSim = new FlywheelSim(
			LinearSystemId.createFlywheelSystem(DCMotor.getFalcon500Foc(1), 0.01, FlyWheelConstants.GEAR_RATIO),
			DCMotor.getFalcon500(1)
		);

		FlywheelSimulation rightFlywheelSimulation = new FlywheelSimulation(rightSim);

		TalonFXConfiguration rightConfiguration = generateSimulationMotorConfig();
		rightConfiguration.Slot0.withKP(9).withKI(6).withKD(0);

		TalonFXMotor rightFlywheel = new TalonFXMotor(
			rightLogPath,
			IDs.TalonFXIDs.RIGHT_FLYWHEEL,
			rightConfiguration,
			generateSysidConfig(),
			rightFlywheelSimulation
		);

		Phoenix6AngleSignal rightVelocitySignal = Phoenix6SignalBuilder.generatePhoenix6Signal(
			rightFlywheel.getMotor().getVelocity(),
			GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
			AngleUnit.ROTATIONS
		);
		Phoenix6DoubleSignal rightCurrentSignal = Phoenix6SignalBuilder
			.generatePhoenix6Signal(rightFlywheel.getMotor().getStatorCurrent(), GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ);
		Phoenix6DoubleSignal rightVoltageSignal = Phoenix6SignalBuilder
			.generatePhoenix6Signal(rightFlywheel.getMotor().getMotorVoltage(), GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ);

		FlywheelSim leftSim = new FlywheelSim(
			LinearSystemId.createFlywheelSystem(DCMotor.getFalcon500Foc(1), 0.01, FlyWheelConstants.GEAR_RATIO),
			DCMotor.getFalcon500(1)
		);

		FlywheelSimulation leftFlywheelSimulation = new FlywheelSimulation(leftSim);

		TalonFXConfiguration leftConfiguration = generateSimulationMotorConfig();
		leftConfiguration.Slot0.withKP(9).withKI(6).withKD(0);

		TalonFXMotor leftFlywheel = new TalonFXMotor(
			leftLogPath,
			IDs.TalonFXIDs.LEFT_FLYWHEEL,
			leftConfiguration,
			generateSysidConfig(),
			leftFlywheelSimulation
		);

		Phoenix6AngleSignal leftVelocitySignal = Phoenix6SignalBuilder
			.generatePhoenix6Signal(leftFlywheel.getMotor().getVelocity(), GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.ROTATIONS);
		Phoenix6DoubleSignal leftCurrentSignal = Phoenix6SignalBuilder
			.generatePhoenix6Signal(leftFlywheel.getMotor().getStatorCurrent(), GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ);
		Phoenix6DoubleSignal leftVoltageSignal = Phoenix6SignalBuilder
			.generatePhoenix6Signal(leftFlywheel.getMotor().getMotorVoltage(), GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ);

		return new FlywheelStuff(
			logPath,
			rightFlywheel,
			leftFlywheel,
			rightVelocityRequest,
			leftVelocityRequest,
			rightVoltageRequest,
			leftVoltageRequest,
			rightVelocitySignal,
			leftVelocitySignal,
			new InputSignal[] {rightCurrentSignal, rightVoltageSignal},
			new InputSignal[] {leftCurrentSignal, leftVoltageSignal}
		);
	}

}
