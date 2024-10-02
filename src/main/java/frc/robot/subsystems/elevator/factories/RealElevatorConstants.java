package frc.robot.subsystems.elevator.factories;

import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.IDs;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.digitalinput.supplied.SuppliedDigitalInput;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.motor.IMotor;
import frc.robot.hardware.motor.sparkmax.BrushlessSparkMAXMotor;
import frc.robot.hardware.motor.sparkmax.SparkMaxWrapper;
import frc.robot.hardware.signal.supplied.SuppliedAngleSignal;
import frc.robot.hardware.signal.supplied.SuppliedDoubleSignal;
import frc.robot.subsystems.elevator.ElevatorStuff;
import frc.utils.AngleUnit;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class RealElevatorConstants {

	private final static double DEBOUNCE_TIME_SECONDS = 0.05;

	private final static Debouncer.DebounceType DEBOUNCE_TYPE = Debouncer.DebounceType.kBoth;

	private final static CANSparkBase.SoftLimitDirection SOFT_LIMIT_DIRECTION = CANSparkBase.SoftLimitDirection.kReverse;

	// TODO: check this later
	private final static float SOFT_LIMIT_VALUE = (float) 0.2;

	// TODO: check this later
	private final static double GEAR_RATIO = 0.3;

	private final static SparkLimitSwitch.Type REVERSE_LIMIT_SWITCH_TYPE = SparkLimitSwitch.Type.kNormallyOpen;

	public static ElevatorStuff generateElevatorStuff(String logPath) {
		SparkMaxWrapper mainSparkMaxWrapper = new SparkMaxWrapper(IDs.ELEVATOR_FIRST_MOTOR);
		SparkMaxWrapper secondarySparkMaxWrapper = new SparkMaxWrapper(IDs.ELEVATOR_SECOND_MOTOR);

		ControllableMotor mainMotor = new BrushlessSparkMAXMotor(logPath, mainSparkMaxWrapper, new SysIdRoutine.Config());

		IMotor secondaryMotor = new BrushlessSparkMAXMotor(logPath, secondarySparkMaxWrapper, new SysIdRoutine.Config());

		Supplier<Double> mainMotorPosition = () -> mainSparkMaxWrapper.getEncoder().getPosition();
		SuppliedAngleSignal mainMotorPositionSignal = new SuppliedAngleSignal("main motor position", mainMotorPosition, AngleUnit.ROTATIONS);
		mainSparkMaxWrapper.setSoftLimit(SOFT_LIMIT_DIRECTION, SOFT_LIMIT_VALUE);
		mainSparkMaxWrapper.getEncoder().setPositionConversionFactor(GEAR_RATIO);

		Supplier<Double> secondaryMotorPosition = () -> secondarySparkMaxWrapper.getEncoder().getPosition();
		SuppliedAngleSignal secondaryMotorPositionSignal = new SuppliedAngleSignal(
			"secondary motor position",
			secondaryMotorPosition,
			AngleUnit.ROTATIONS
		);
		secondarySparkMaxWrapper.setSoftLimit(SOFT_LIMIT_DIRECTION, SOFT_LIMIT_VALUE);
		secondarySparkMaxWrapper.getEncoder().setPositionConversionFactor(GEAR_RATIO);

		Supplier<Double> motorsVoltage = () -> (mainSparkMaxWrapper.getBusVoltage() * mainSparkMaxWrapper.getAppliedOutput());
		SuppliedDoubleSignal motorsVoltageSignal = new SuppliedDoubleSignal("main motor voltage", motorsVoltage);

		secondarySparkMaxWrapper.follow(mainSparkMaxWrapper);

		BooleanSupplier inLimitSwitch = () -> mainSparkMaxWrapper.getReverseLimitSwitch(REVERSE_LIMIT_SWITCH_TYPE).isPressed();
		mainSparkMaxWrapper.getReverseLimitSwitch(REVERSE_LIMIT_SWITCH_TYPE).enableLimitSwitch(true);
		IDigitalInput limitSwitchDigitalInputs = new SuppliedDigitalInput(inLimitSwitch, DEBOUNCE_TYPE, DEBOUNCE_TIME_SECONDS);

		return new ElevatorStuff(
			logPath,
			mainMotor,
			motorsVoltageSignal,
			mainMotorPositionSignal,
			secondaryMotorPositionSignal,
			limitSwitchDigitalInputs
		);
	}

}
