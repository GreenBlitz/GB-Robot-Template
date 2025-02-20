package frc.robot.subsystems.climb.lifter.factory;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.IDs;
import frc.robot.RobotConstants;
import frc.robot.hardware.mechanisms.wpilib.SimpleMotorSimulation;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.hardware.phoenix6.signal.Phoenix6AngleSignal;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.robot.subsystems.climb.lifter.Lifter;
import frc.utils.math.AngleUnit;

public class Falcon500LifterBuilder {

	private static final int DIGITAL_INPUT_CHANNEL = 9;
	private static final double DEBOUNCE_TIME = 0.05;

	private static final int NUMBER_OF_MOTORS = 1;

	private static final double CURRENT_LIMIT = 40;

	private static final boolean SET_BRAKE = true;
	private static final boolean INVERTED = false;
	private static final double GEAR_RATIO = 25.0 / 6.0;
	private static final double SENSOR_TO_MECHANISM_RATIO = 7 * GEAR_RATIO;
	private static final double MOMENT_OF_INERTIA = 0.001;

	private static TalonFXConfiguration generateMotorConfiguration() {
		TalonFXConfiguration configuration = new TalonFXConfiguration();

		configuration.Feedback.SensorToMechanismRatio = SENSOR_TO_MECHANISM_RATIO;
		configuration.MotorOutput.Inverted = INVERTED ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive;
		configuration.CurrentLimits.StatorCurrentLimit = CURRENT_LIMIT;
		configuration.CurrentLimits.StatorCurrentLimitEnable = true;

		return configuration;
	}

	protected static Lifter createLifter(String logPath) {
		SimpleMotorSimulation simulation = new SimpleMotorSimulation(
			new DCMotorSim(
				LinearSystemId.createDCMotorSystem(DCMotor.getFalcon500Foc(NUMBER_OF_MOTORS), MOMENT_OF_INERTIA, 1 / GEAR_RATIO),
				DCMotor.getFalcon500Foc(NUMBER_OF_MOTORS)
			)
		);
		TalonFXMotor lifter = new TalonFXMotor(logPath, IDs.TalonFXIDs.LIFTER, new SysIdRoutine.Config(), simulation);
		lifter.applyConfiguration(generateMotorConfiguration());
		lifter.setBrake(SET_BRAKE);

		Phoenix6AngleSignal positionSignal = Phoenix6SignalBuilder
			.build(lifter.getDevice().getPosition(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.ROTATIONS);

		return new Lifter(logPath, lifter, positionSignal);
	}

}
