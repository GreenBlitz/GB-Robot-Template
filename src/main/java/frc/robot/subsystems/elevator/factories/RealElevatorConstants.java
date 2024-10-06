package frc.robot.subsystems.elevator.factories;

import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.IDs;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.digitalinput.supplied.SuppliedDigitalInput;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.motor.sparkmax.BrushlessSparkMAXMotor;
import frc.robot.hardware.motor.sparkmax.SparkMaxWrapper;
import frc.robot.hardware.request.cansparkmax.SparkMaxAngleRequest;
import frc.robot.hardware.signal.supplied.SuppliedAngleSignal;
import frc.robot.hardware.signal.supplied.SuppliedDoubleSignal;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorMotorStuff;
import frc.robot.subsystems.elevator.ElevatorStuff;
import frc.utils.AngleUnit;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class RealElevatorConstants {

	private final static double DEBOUNCE_TIME_SECONDS = 0.05;

	private final static Debouncer.DebounceType DEBOUNCE_TYPE = Debouncer.DebounceType.kBoth;

	private final static CANSparkBase.SoftLimitDirection SOFT_LIMIT_DIRECTION = CANSparkBase.SoftLimitDirection.kReverse;

	public static final int ELEVATOR_PID_SLOT = 0;

	// TODO: check this later
	private final static Rotation2d REVERSE_SOFT_LIMIT_VALUE = Rotation2d.fromRotations(0);

	private final static double ROTATIONS_TO_METERS_CONVERTION_RATIO = 1;

	private final static SparkLimitSwitch.Type REVERSE_LIMIT_SWITCH_TYPE = SparkLimitSwitch.Type.kNormallyOpen;

	public static ElevatorFeedforward FEEDFORWARD_CALCULATOR = new ElevatorFeedforward(0, 0, 0, 0);

	public static double feedforwardCalculation(Rotation2d velocity) {
		return RealElevatorConstants.FEEDFORWARD_CALCULATOR.calculate(velocity.getRadians());
	}

	public static void configureMotor(SparkMaxWrapper sparkMaxWrapper) {
		sparkMaxWrapper.setSoftLimit(SOFT_LIMIT_DIRECTION, (float) REVERSE_SOFT_LIMIT_VALUE.getRotations());
		sparkMaxWrapper.enableSoftLimit(SOFT_LIMIT_DIRECTION, true);
		sparkMaxWrapper.getEncoder().setPositionConversionFactor(ElevatorConstants.GEAR_RATIO);
		sparkMaxWrapper.getEncoder().setVelocityConversionFactor(ElevatorConstants.GEAR_RATIO);
		sparkMaxWrapper.getPIDController().setP(1);
		sparkMaxWrapper.getPIDController().setI(0);
		sparkMaxWrapper.getPIDController().setD(0);
	}

	public static ElevatorMotorStuff generateMotor(String logPath, String motorName, SparkMaxWrapper sparkMaxWrapper) {
		ControllableMotor motor = new BrushlessSparkMAXMotor(logPath, sparkMaxWrapper, new SysIdRoutine.Config());

		Supplier<Double> motorPosition = () -> sparkMaxWrapper.getEncoder().getPosition();
		SuppliedAngleSignal motorPositionSignal = new SuppliedAngleSignal(motorName + " angle", motorPosition, AngleUnit.ROTATIONS);

		Supplier<Double> motorsVoltage = sparkMaxWrapper::getVoltage;
		SuppliedDoubleSignal motorVoltageSignal = new SuppliedDoubleSignal("main motor voltage", motorsVoltage);

		return new ElevatorMotorStuff(motor, motorVoltageSignal, motorPositionSignal);
	}

	public static ElevatorStuff generateElevatorStuff(String logPath) {
		SparkMaxWrapper frontMotorWrapper = new SparkMaxWrapper(IDs.CANSparkMAXIDs.ELEVATOR_FRONT_MOTOR);
		SparkMaxWrapper backwardMotorWrapper = new SparkMaxWrapper(IDs.CANSparkMAXIDs.ELEVATOR_SECOND_MOTOR);
		configureMotor(frontMotorWrapper);
		configureMotor(backwardMotorWrapper);

		ElevatorMotorStuff frontMotorStuff = generateMotor(logPath + "frontMotor", "first motor", frontMotorWrapper);
		ElevatorMotorStuff backwardMotorStuff = generateMotor(logPath + "secondMotor", "second motor", backwardMotorWrapper);

		BooleanSupplier atLimitSwitch = () -> frontMotorWrapper.getReverseLimitSwitch(REVERSE_LIMIT_SWITCH_TYPE).isPressed();
		frontMotorWrapper.getReverseLimitSwitch(REVERSE_LIMIT_SWITCH_TYPE).enableLimitSwitch(true);
		IDigitalInput limitSwitch = new SuppliedDigitalInput(atLimitSwitch, DEBOUNCE_TYPE, DEBOUNCE_TIME_SECONDS);

		SparkMaxAngleRequest angleRequest = new SparkMaxAngleRequest(
			Rotation2d.fromRotations(0),
			SparkMaxAngleRequest.SparkAngleRequestType.POSITION,
			ELEVATOR_PID_SLOT,
			RealElevatorConstants::feedforwardCalculation
		);

		return new ElevatorStuff(logPath, angleRequest, limitSwitch, frontMotorStuff, backwardMotorStuff, ROTATIONS_TO_METERS_CONVERTION_RATIO);
	}

}
