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

	private static final double DEBOUNCE_TIME_SECONDS = 0.05;

	private static final Debouncer.DebounceType DEBOUNCE_TYPE = Debouncer.DebounceType.kBoth;

	private static final CANSparkBase.SoftLimitDirection SOFT_LIMIT_DIRECTION = CANSparkBase.SoftLimitDirection.kReverse;

	private static final int ELEVATOR_PID_SLOT = 0;

	private static final Rotation2d REVERSE_SOFT_LIMIT_VALUE = Rotation2d.fromRotations(0);

	private static final double MOTOR_ROTATIONS_TO_METERS_CONVERTION_RATIO = 1;

	private static final SparkLimitSwitch.Type REVERSE_LIMIT_SWITCH_TYPE = SparkLimitSwitch.Type.kNormallyOpen;

	private static final ElevatorFeedforward FEEDFORWARD_CALCULATOR = new ElevatorFeedforward(0, 0, 0, 0);

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

	public static ElevatorMotorStuff generateMotorStuff(String logPath, String motorName, SparkMaxWrapper sparkMaxWrapper) {
		configureMotor(sparkMaxWrapper);

		ControllableMotor motor = new BrushlessSparkMAXMotor(logPath, sparkMaxWrapper, new SysIdRoutine.Config());

		Supplier<Double> motorPosition = () -> sparkMaxWrapper.getEncoder().getPosition();
		SuppliedAngleSignal motorPositionSignal = new SuppliedAngleSignal(motorName + " angle", motorPosition, AngleUnit.ROTATIONS);

		Supplier<Double> motorsVoltage = sparkMaxWrapper::getVoltage;
		SuppliedDoubleSignal motorVoltageSignal = new SuppliedDoubleSignal(motorName + " voltage", motorsVoltage);

		return new ElevatorMotorStuff(motor, motorVoltageSignal, motorPositionSignal);
	}

	public static ElevatorStuff generateElevatorStuff(String logPath) {
		SparkMaxWrapper frontMotorWrapper = new SparkMaxWrapper(IDs.CANSparkMAXIDs.ELEVATOR_FRONT);
		SparkMaxWrapper backMotorWrapper = new SparkMaxWrapper(IDs.CANSparkMAXIDs.ELEVATOR_BACK);

		ElevatorMotorStuff frontMotorStuff = generateMotorStuff(logPath + "front motor", "front motor", frontMotorWrapper);
		ElevatorMotorStuff backMotorStuff = generateMotorStuff(logPath + "back motor", "back motor", backMotorWrapper);

		BooleanSupplier atLimitSwitch = () -> frontMotorWrapper.getReverseLimitSwitch(REVERSE_LIMIT_SWITCH_TYPE).isPressed();
		frontMotorWrapper.getReverseLimitSwitch(REVERSE_LIMIT_SWITCH_TYPE).enableLimitSwitch(true);
		IDigitalInput limitSwitch = new SuppliedDigitalInput(atLimitSwitch, DEBOUNCE_TYPE, DEBOUNCE_TIME_SECONDS);

		SparkMaxAngleRequest angleRequest = new SparkMaxAngleRequest(
			Rotation2d.fromRotations(0),
			SparkMaxAngleRequest.SparkAngleRequestType.POSITION,
			ELEVATOR_PID_SLOT,
			RealElevatorConstants::feedforwardCalculation
		);

		return new ElevatorStuff(
			logPath,
			angleRequest,
			limitSwitch,
			frontMotorStuff,
			backMotorStuff,
			MOTOR_ROTATIONS_TO_METERS_CONVERTION_RATIO
		);
	}

}
