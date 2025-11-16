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
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.hardware.mechanisms.wpilib.SimpleMotorSimulation;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.motors.TalonFXFollowerConfig;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.hardware.phoenix6.request.Phoenix6Request;
import frc.robot.hardware.phoenix6.request.Phoenix6RequestBuilder;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.utils.AngleUnit;
import frc.utils.battery.BatteryUtil;

public class TalonFXRollerBuilder {

	public static Roller generate(
		String logPath,
		Phoenix6DeviceID id,
		double gearRatio,
		int currentLimit,
		Rotation2d tolerance,
		double kP,
		double kI,
		double kD
	) {
		SimpleMotorSimulation rollerSimulation = new SimpleMotorSimulation(
			new DCMotorSim(
				LinearSystemId
					.createDCMotorSystem(DCMotor.getKrakenX60(RollerConstants.NUMBER_OF_MOTORS), RollerConstants.MOMENT_OF_INERTIA, gearRatio),
				DCMotor.getKrakenX60(RollerConstants.NUMBER_OF_MOTORS)
			)

		);
		TalonFXMotor roller = new TalonFXMotor(logPath, id, new TalonFXFollowerConfig(), new SysIdRoutine.Config(), rollerSimulation);

		roller.applyConfiguration(configRoller(gearRatio, currentLimit, kP, kI, kD));

		InputSignal<Rotation2d> positionSignal = Phoenix6SignalBuilder.build(
			roller.getDevice().getPosition(),
			roller.getDevice().getVelocity(),
			RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
			AngleUnit.ROTATIONS,
			BusChain.ROBORIO
		);
		InputSignal<Double> currentSignal = Phoenix6SignalBuilder
			.build(roller.getDevice().getStatorCurrent(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, id.busChain());
		InputSignal<Double> voltageSignal = Phoenix6SignalBuilder
			.build(roller.getDevice().getMotorVoltage(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, id.busChain());

		Phoenix6Request<Rotation2d> PositionRequest = Phoenix6RequestBuilder.build(new PositionVoltage(0), 0, true);
		Phoenix6Request<Double> VoltageRequest = Phoenix6RequestBuilder.build(new VoltageOut(0), true);

		return new Roller(logPath, roller, voltageSignal, positionSignal, currentSignal, VoltageRequest, PositionRequest, tolerance);
	}

	public static TalonFXConfiguration configRoller(double gearRatio, int currentLimit, double kP, double kI, double kD) {
		TalonFXConfiguration configs = new TalonFXConfiguration();
		configs.CurrentLimits.StatorCurrentLimit = currentLimit;
		configs.CurrentLimits.StatorCurrentLimitEnable = true;
		configs.Feedback.SensorToMechanismRatio = gearRatio;
		configs.Voltage.PeakForwardVoltage = BatteryUtil.DEFAULT_VOLTAGE;
		configs.Voltage.PeakReverseVoltage = BatteryUtil.DEFAULT_VOLTAGE;
		configs.Slot0.kI = kI;
		configs.Slot0.kP = kP;
		configs.Slot0.kD = kD;
		return (configs);
	}

}
