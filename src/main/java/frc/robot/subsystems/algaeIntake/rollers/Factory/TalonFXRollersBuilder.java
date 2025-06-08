package frc.robot.subsystems.algaeIntake.rollers.Factory;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.IDs;
import frc.robot.RobotConstants;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.hardware.mechanisms.wpilib.SimpleMotorSimulation;
import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.robot.subsystems.algaeIntake.rollers.Rollers;

public class TalonFXRollersBuilder {

	private static final int NUM_MOTORS = 1;
	private static final double MOMENT_OF_INERTIA = 0.02;
	private static final double GEAR_RATIO = 1 / 1;
	private static final boolean IS_INVERTED = false;


	private static TalonFXMotor generateMotor(String logPath) {
		SimpleMotorSimulation sim = new SimpleMotorSimulation(
			new DCMotorSim(
				LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(NUM_MOTORS), MOMENT_OF_INERTIA, GEAR_RATIO),
				DCMotor.getKrakenX60(NUM_MOTORS)
			)
		);

		return new TalonFXMotor(logPath + "/Motor", IDs.TalonFXIDs.ROLLERS, new SysIdRoutine.Config(), sim);
	}

	private static TalonFXConfiguration generateMotorConfig() {
		TalonFXConfiguration config = new TalonFXConfiguration();

		config.MotorOutput.Inverted = IS_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

		config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

		config.CurrentLimits.StatorCurrentLimit = 40;
		config.CurrentLimits.StatorCurrentLimitEnable = true;

		return config;
	}

	public static Rollers generate(String logPath) {
		TalonFXMotor rollers = generateMotor(logPath);
		rollers.applyConfiguration(generateMotorConfig());

		InputSignal<Double> voltageSignal = Phoenix6SignalBuilder
			.build(rollers.getDevice().getMotorVoltage(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, BusChain.ROBORIO);

		CANrange canRange = new CANrange(IDs.CANRangeIDs.ROLLERS_CAN_RANGE, BusChain.ROBORIO.getChainName());

		return new Rollers(logPath, rollers, voltageSignal, canRange);
	}

}
