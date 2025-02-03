package frc.robot.subsystems.elevator.factory;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.IDs;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.digitalinput.channeled.ChanneledDigitalInput;
import frc.robot.hardware.digitalinput.chooser.ChooserDigitalInput;
import frc.robot.hardware.mechanisms.wpilib.ElevatorSimulation;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.hardware.phoenix6.request.Phoenix6FeedForwardRequest;
import frc.robot.hardware.phoenix6.request.Phoenix6Request;
import frc.robot.hardware.phoenix6.request.Phoenix6RequestBuilder;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.records.ElevatorMotorSignals;
import frc.utils.math.AngleUnit;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Second;

public class KrakenX60ElevatorBuilder {

	private static final int LIMIT_SWITCH_CHANNEL = 0;
	private static final double LIMIT_SWITCH_DEBOUNCE_TIME = 0.04;
	private static final double CURRENT_LIMIT = 40;
	private static final boolean CURRENT_LIMIT_ENABLE = true;
	private static final boolean SOFT_LIMIT_ENABLE = true;
	private static final boolean IS_FIRST_MOTOR_INVERTED = true;
	private static final boolean IS_SECOND_MOTOR_INVERTED = true;

	private static final double REAL_KP = 1;
	private static final double REAL_KI = 0;
	private static final double REAL_KD = 0;

	private static final double SIMULATION_KP = 1;
	private static final double SIMULATION_KI = 0;
	private static final double SIMULATION_KD = 0.05;
	private static final int NUMBER_OF_MOTORS = 2;
	private static final double STARTING_HEIGHT_METERS = 0;

	private static final Velocity<VoltageUnit> CONFIG_RAMP_RATE = Volts.of(1).per(Second);
	private static final Voltage CONFIG_STEP_VOLTAGE = Volts.of(7);
	private static final Time CONFIG_TIMEOUT = Seconds.of(10);

	private static SysIdRoutine.Config generateSysidConfig() {
		return new SysIdRoutine.Config(
			CONFIG_RAMP_RATE,
			CONFIG_STEP_VOLTAGE,
			CONFIG_TIMEOUT,
			(state) -> Logger.recordOutput("state", state.toString())
		);
	}

	private static TalonFXConfiguration generateConfiguration(boolean inverted) {
		TalonFXConfiguration configuration = new TalonFXConfiguration();
		if (Robot.ROBOT_TYPE.isReal()) {
			configuration.Slot0.kP = REAL_KP;
			configuration.Slot0.kI = REAL_KI;
			configuration.Slot0.kD = REAL_KD;
		} else {
			configuration.Slot0.kP = SIMULATION_KP;
			configuration.Slot0.kI = SIMULATION_KI;
			configuration.Slot0.kD = SIMULATION_KD;
		}
		configuration.CurrentLimits.StatorCurrentLimit = CURRENT_LIMIT;
		configuration.CurrentLimits.StatorCurrentLimitEnable = CURRENT_LIMIT_ENABLE;
		configuration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Elevator
			.convertMetersToRotations(ElevatorConstants.REVERSE_SOFT_LIMIT_VALUE_METERS)
			.getRotations();
		configuration.SoftwareLimitSwitch.ReverseSoftLimitEnable = SOFT_LIMIT_ENABLE;
		configuration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Elevator
			.convertMetersToRotations(ElevatorConstants.FORWARD_SOFT_LIMIT_VALUE_METERS)
			.getRotations();
		configuration.SoftwareLimitSwitch.ForwardSoftLimitEnable = SOFT_LIMIT_ENABLE;
		configuration.MotorOutput.Inverted = inverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
		configuration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		return configuration;
	}

	private static IDigitalInput generateDigitalInput() {
		return Robot.ROBOT_TYPE.isSimulation()
			? new ChooserDigitalInput("ElevatorLimitSwitch")
			: new ChanneledDigitalInput(new DigitalInput(LIMIT_SWITCH_CHANNEL), new Debouncer(LIMIT_SWITCH_DEBOUNCE_TIME));
	}

	private static ElevatorMotorSignals createSignals(TalonFXMotor motor) {
		return new ElevatorMotorSignals(
			Phoenix6SignalBuilder.build(motor.getDevice().getPosition(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.ROTATIONS),
			Phoenix6SignalBuilder.build(motor.getDevice().getMotorVoltage(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ)
		);
	}

	private static ElevatorSimulation generateSimulation() {
		return new ElevatorSimulation(
			new ElevatorSim(
				LinearSystemId.createElevatorSystem(
					DCMotor.getKrakenX60Foc(NUMBER_OF_MOTORS),
					ElevatorConstants.MASS_KG,
					ElevatorConstants.DRUM_RADIUS_METERS,
					ElevatorConstants.GEAR_RATIO
				),
				DCMotor.getKrakenX60Foc(NUMBER_OF_MOTORS),
				ElevatorConstants.MINIMUM_HEIGHT_METERS,
				ElevatorConstants.MAXIMUM_HEIGHT_METERS,
				false,
				STARTING_HEIGHT_METERS
			),
			ElevatorConstants.DRUM_DIAMETER_METERS,
			ElevatorConstants.GEAR_RATIO
		);
	}

	public static Elevator createRealElevator(String logPath) {
		TalonFXMotor rightMotor = new TalonFXMotor(logPath, IDs.TalonFXIDs.ELEVATOR_RIGHT, generateSysidConfig());
		rightMotor.applyConfiguration(generateConfiguration(IS_FIRST_MOTOR_INVERTED));

		TalonFXMotor leftMotor = new TalonFXMotor(logPath, IDs.TalonFXIDs.ELEVATOR_LEFT, generateSysidConfig());
		leftMotor.applyConfiguration(generateConfiguration(IS_SECOND_MOTOR_INVERTED));

		return create(logPath, rightMotor, leftMotor);
	}

	public static Elevator createSimulationElevator(String logPath) {
		TalonFXMotor rightMotor = new TalonFXMotor(logPath, IDs.TalonFXIDs.ELEVATOR_RIGHT, generateSysidConfig(), generateSimulation());
		rightMotor.applyConfiguration(generateConfiguration(IS_FIRST_MOTOR_INVERTED));

		return create(logPath, rightMotor, rightMotor);
	}

	private static Elevator create(String logPath, TalonFXMotor rightMotor, TalonFXMotor leftMotor) {
		IDigitalInput digitalInput = generateDigitalInput();

		Phoenix6FeedForwardRequest positionRequest = Phoenix6RequestBuilder.build(
			new DynamicMotionMagicVoltage(
				0,
				Elevator.convertMetersToRotations(ElevatorConstants.CRUISE_VELOCITY_METERS_PER_SECOND).getRotations(),
				Elevator.convertMetersToRotations(ElevatorConstants.ACCELERATION_METERS_PER_SECOND_SQUARED).getRotations(),
				0
			),
			0,
			true
		);
		Phoenix6Request<Double> voltageRequest = Phoenix6RequestBuilder.build(new VoltageOut(0), true);

		return new Elevator(
			logPath,
			rightMotor,
			createSignals(rightMotor),
			leftMotor,
			createSignals(leftMotor),
			positionRequest,
			voltageRequest,
			digitalInput
		);
	}

}
