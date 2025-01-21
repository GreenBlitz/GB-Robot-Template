package frc.robot.subsystems.elevator.factory;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
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
import frc.robot.hardware.phoenix6.request.Phoenix6Request;
import frc.robot.hardware.phoenix6.request.Phoenix6RequestBuilder;
import frc.robot.hardware.phoenix6.signal.Phoenix6SignalBuilder;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.records.ElevatorMotorSignals;
import frc.utils.AngleUnit;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.*;

public class ElevatorFactory {

	private static final int LIMIT_SWITCH_CHANNEL = 0;
	private static final double LIMIT_SWITCH_DEBOUNCE_TIME = 0.04;
	private static final double CURRENT_LIMIT = 40;
	private static final boolean CURRENT_LIMIT_ENABLE = true;
	private static final boolean SOFT_LIMIT_ENABLE = true;

	private static final double REAL_KP = 1;
	private static final double REAL_KI = 0;
	private static final double REAL_KD = 0;

	private static final double SIMULATION_KP = 1;
	private static final double SIMULATION_KI = 0;
	private static final double SIMULATION_KD = 0;
	public static final int NUMBER_OF_MOTORS = 2;
	public static final double STARTING_HEIGHT_METERS = 0;

	private static SysIdRoutine.Config generateSysidConfig() {
		return new SysIdRoutine.Config(
			Volts.of(1).per(Second),
			Volts.of(7),
			Seconds.of(10),
			(state) -> Logger.recordOutput("state", state.toString())
		);
	}

	private static TalonFXConfiguration generateConfiguration() {
		TalonFXConfiguration configuration = new TalonFXConfiguration();
		if (Robot.ROBOT_TYPE.isReal()) {
			configuration.Slot0.withKP(REAL_KP).withKI(REAL_KI).withKD(REAL_KD);
		} else {
			configuration.Slot0.withKP(SIMULATION_KP).withKI(SIMULATION_KI).withKD(SIMULATION_KD);
		}
		configuration.CurrentLimits.StatorCurrentLimit = CURRENT_LIMIT;
		configuration.CurrentLimits.StatorCurrentLimitEnable = CURRENT_LIMIT_ENABLE;
		configuration.SoftwareLimitSwitch
			.withReverseSoftLimitThreshold(Elevator.convertMetersToRotations(ElevatorConstants.REVERSE_SOFT_LIMIT_VALUE_METERS).getRotations());
		configuration.SoftwareLimitSwitch.withReverseSoftLimitEnable(SOFT_LIMIT_ENABLE);
		configuration.SoftwareLimitSwitch
			.withForwardSoftLimitThreshold(Elevator.convertMetersToRotations(ElevatorConstants.FORWARD_SOFT_LIMIT_VALUE_METERS).getRotations());
		configuration.SoftwareLimitSwitch.withForwardSoftLimitEnable(SOFT_LIMIT_ENABLE);
		return configuration;
	}

	private static IDigitalInput generateDigitalInput() {
		return Robot.ROBOT_TYPE.isSimulation()
			? new ChooserDigitalInput("ElevatorLimitSwitch")
			: new ChanneledDigitalInput(new DigitalInput(LIMIT_SWITCH_CHANNEL), new Debouncer(LIMIT_SWITCH_DEBOUNCE_TIME));
	}

	private static ElevatorMotorSignals createSignals(TalonFXMotor motor) {
		return new ElevatorMotorSignals(
			Phoenix6SignalBuilder
				.generatePhoenix6Signal(motor.getDevice().getPosition(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, AngleUnit.ROTATIONS),
			Phoenix6SignalBuilder.generatePhoenix6Signal(motor.getDevice().getMotorVoltage(), RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ)
		);
	}

	private static ElevatorSimulation generateSimulation() {
		return new ElevatorSimulation(
			new ElevatorSim(
				LinearSystemId.createElevatorSystem(
					DCMotor.getKrakenX60Foc(NUMBER_OF_MOTORS),
					ElevatorConstants.MASS_KG,
					ElevatorConstants.ELEVATOR_RADIUS_METERS,
					ElevatorConstants.GEAR_RATIO
				),
				DCMotor.getKrakenX60Foc(NUMBER_OF_MOTORS),
				ElevatorConstants.MINIMUM_HEIGHT_METERS,
				ElevatorConstants.MAXIMUM_HEIGHT_METERS,
				false,
				STARTING_HEIGHT_METERS
			),
			ElevatorConstants.DRUM_RADIUS_METERS,
			ElevatorConstants.GEAR_RATIO
		);
	}

	private static TalonFXMotor generateFirstMotor(String logPath, ElevatorSimulation simulation) {
		return new TalonFXMotor(logPath, IDs.Phoenix6IDs.ELEVATOR_FIRST_MOTOR_ID, generateSysidConfig(), simulation);
	}

	private static TalonFXMotor generateSecondMotor(String logPath) {
		return new TalonFXMotor(logPath, IDs.Phoenix6IDs.ELEVATOR_SECOND_MOTOR_ID, generateSysidConfig());
	}

	public static Elevator create(String logPath) {
		ElevatorSimulation elevatorSimulation = generateSimulation();

		TalonFXMotor firstMotor = generateFirstMotor(logPath + "FirstMotor/", elevatorSimulation);
		firstMotor.applyConfiguration(generateConfiguration());

		TalonFXMotor secondMotor = generateSecondMotor(logPath + "SecondMotor/");
		secondMotor.applyConfiguration(generateConfiguration());

		IDigitalInput digitalInput = generateDigitalInput();

		Phoenix6Request<Rotation2d> positionRequest = Phoenix6RequestBuilder.build(new PositionVoltage(0));
		Phoenix6Request<Double> voltageRequest = Phoenix6RequestBuilder.build(new VoltageOut(0));

		return switch (Robot.ROBOT_TYPE) {
			case REAL ->
				new Elevator(
					logPath,
					firstMotor,
					createSignals(firstMotor),
					secondMotor,
					createSignals(secondMotor),
					positionRequest,
					voltageRequest,
					digitalInput
				);
			case SIMULATION ->
				new Elevator(
					logPath,
					firstMotor,
					createSignals(firstMotor),
					firstMotor,
					createSignals(firstMotor),
					positionRequest,
					voltageRequest,
					digitalInput
				);
		};
	}

}
