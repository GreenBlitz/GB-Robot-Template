package frc.robot.subsystems.swerve.factories.modules.drive;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
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
import frc.robot.subsystems.swerve.module.records.DriveRequests;
import frc.robot.subsystems.swerve.module.records.DriveSignals;
import frc.utils.AngleUnit;

class Falcon500DriveBuilder {

	private static final double SLIP_CURRENT = 60;
	private static final double GEAR_RATIO = 6.12;

	private static SysIdRoutine.Config buildSysidConfig() {
		return new SysIdRoutine.Config(
			Units.Volts.of(0.5).per(Units.Second),
			Units.Volts.of(2),
			null,
			state -> SignalLogger.writeString("state", state.toString())
		);
	}

	private static SimpleMotorSimulation buildMechanismSimulation() {
		double momentOfInertiaMetersSquared = 0.001;
		return new SimpleMotorSimulation(
			new DCMotorSim(
				LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500Foc(1), momentOfInertiaMetersSquared, GEAR_RATIO),
				DCMotor.getFalcon500Foc(1)
			)
		);
	}

	private static TalonFXConfiguration buildMotorConfig(boolean inverted) {
		TalonFXConfiguration driveConfig = new TalonFXConfiguration();

		driveConfig.MotorOutput.Inverted = inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

		driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		driveConfig.Feedback.SensorToMechanismRatio = GEAR_RATIO;

		driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = SLIP_CURRENT;
		driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -SLIP_CURRENT;
		driveConfig.CurrentLimits.StatorCurrentLimit = SLIP_CURRENT;
		driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;

		driveConfig.Slot0.kS = 0.21549;
		driveConfig.Slot0.kV = 0.72124;
		driveConfig.Slot0.kA = 0.11218;
		driveConfig.Slot0.kP = 1.5;
		driveConfig.Slot0.kI = 0;
		driveConfig.Slot0.kD = 0;

		return driveConfig;
	}

	static ControllableMotor buildDrive(String logPath, Phoenix6DeviceID deviceID, boolean inverted) {
		TalonFXMotor drive = new TalonFXMotor(logPath, deviceID, buildSysidConfig(), buildMechanismSimulation());
		drive.applyConfiguration(buildMotorConfig(inverted));
		return drive;
	}

	static DriveRequests buildRequests() {
		return new DriveRequests(
			Phoenix6RequestBuilder.build(new VelocityVoltage(0).withEnableFOC(true)),
			Phoenix6RequestBuilder.build(new VoltageOut(0).withEnableFOC(true))
		);
	}

	static DriveSignals buildSignals(TalonFXMotor drive) {
		Phoenix6DoubleSignal voltageSignal = Phoenix6SignalBuilder
			.build(drive.getDevice().getMotorVoltage(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ);
		Phoenix6DoubleSignal currentSignal = Phoenix6SignalBuilder
			.build(drive.getDevice().getStatorCurrent(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ);
		Phoenix6AngleSignal velocitySignal = Phoenix6SignalBuilder
			.build(drive.getDevice().getVelocity(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.ROTATIONS);
		Phoenix6LatencySignal positionSignal = Phoenix6SignalBuilder
			.build(drive.getDevice().getPosition(), velocitySignal, RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.ROTATIONS);

		return new DriveSignals(positionSignal, velocitySignal, currentSignal, voltageSignal);
	}

}