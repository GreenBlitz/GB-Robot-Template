package frc.robot.subsystems.roller;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
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
import frc.robot.hardware.phoenix6.signal.Phoenix6DoubleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6LatencySignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.utils.AngleUnit;

public class TalonFXRollerBuilder {

	public static Roller createTalonFXMotorRoller(
		String logPath,
		Phoenix6DeviceID rotorID,
		double gearRatio,
		int currentLimiting,
		Rotation2d tolerance,
		double KP,
		double KI,
		double KD
	) {
		SimpleMotorSimulation rollerSimulation = new SimpleMotorSimulation(
			new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, gearRatio), DCMotor.getKrakenX60(1))

		);
		TalonFXMotor roller = new TalonFXMotor(logPath, rotorID, new TalonFXFollowerConfig(), new SysIdRoutine.Config(), rollerSimulation);
		roller.applyConfiguration(configRoller(gearRatio, currentLimiting, KP, KI, KD));
		Phoenix6LatencySignal latencySignal = Phoenix6SignalBuilder.build(
			roller.getDevice().getPosition(),
			roller.getDevice().getVelocity(),
			RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
			AngleUnit.ROTATIONS,
			BusChain.ROBORIO
		);
		Phoenix6DoubleSignal currentSignal = Phoenix6SignalBuilder
			.build(roller.getDevice().getStatorCurrent(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, BusChain.ROBORIO);
		Phoenix6DoubleSignal voltageSignal = Phoenix6SignalBuilder
			.build(roller.getDevice().getMotorVoltage(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, BusChain.ROBORIO);
		Phoenix6Request<Rotation2d> goToPositionRequest = Phoenix6RequestBuilder.build(new PositionVoltage(0), 0, true);
		Phoenix6Request<Double> setVoltageRequest = Phoenix6RequestBuilder.build(new VoltageOut(0), true);
		return new Roller(logPath, roller, voltageSignal, currentSignal, latencySignal, setVoltageRequest, goToPositionRequest, tolerance);
	}

	public static TalonFXConfiguration configRoller(double gearRatio, int currentLimiting, double KP, double KI, double KD) {
		TalonFXConfiguration configs = new TalonFXConfiguration();
		configs.CurrentLimits.StatorCurrentLimit = currentLimiting;
		configs.CurrentLimits.StatorCurrentLimitEnable = true;
		configs.Feedback.SensorToMechanismRatio = gearRatio;
		configs.Voltage.PeakForwardVoltage = RollerConstants.MAX_VOLTAGE;
		configs.Voltage.PeakReverseVoltage = RollerConstants.MAX_VOLTAGE;
		configs.Slot0.kI = KI;
		configs.Slot0.kP = KP;
		configs.Slot0.kD = KD;
		return (configs);
	}

}
