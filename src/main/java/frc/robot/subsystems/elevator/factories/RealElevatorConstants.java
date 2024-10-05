package frc.robot.subsystems.elevator.factories;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
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
import frc.robot.subsystems.elevator.ElevatorStuff;
import frc.utils.AngleUnit;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class RealElevatorConstants {

	private final static double DEBOUNCE_TIME_SECONDS = 0.05;

	private final static Debouncer.DebounceType DEBOUNCE_TYPE = Debouncer.DebounceType.kBoth;

	private final static CANSparkBase.SoftLimitDirection SOFT_LIMIT_DIRECTION = CANSparkBase.SoftLimitDirection.kReverse;

	// TODO: check this later
	private final static Rotation2d REVERSE_SOFT_LIMIT_VALUE = Rotation2d.fromRotations(0);

	private final static SparkLimitSwitch.Type REVERSE_LIMIT_SWITCH_TYPE = SparkLimitSwitch.Type.kNormallyOpen;

	public static ElevatorFeedforward FEEDFORWARD_CALCULATOR = new ElevatorFeedforward(0, 0, 0, 0);

	public static double feedforwardCalculation(Rotation2d velocity) {
		return RealElevatorConstants.FEEDFORWARD_CALCULATOR.calculate(velocity.getRadians());
	}

	public static ElevatorStuff generateElevatorStuff(String logPath) {
		SparkMaxWrapper mainSparkMaxWrapper = new SparkMaxWrapper(IDs.CANSparkMAXIDs.ELEVATOR_FIRST_MOTOR);
		SparkMaxWrapper secondarySparkMaxWrapper = new SparkMaxWrapper(IDs.CANSparkMAXIDs.ELEVATOR_SECOND_MOTOR);

		ControllableMotor mainMotor = new BrushlessSparkMAXMotor(logPath, mainSparkMaxWrapper, new SysIdRoutine.Config());

		Supplier<Double> mainMotorPosition = () -> mainSparkMaxWrapper.getEncoder().getPosition();
		SuppliedAngleSignal mainMotorPositionSignal = new SuppliedAngleSignal("main motor angle", mainMotorPosition, AngleUnit.ROTATIONS);
		mainSparkMaxWrapper.setSoftLimit(SOFT_LIMIT_DIRECTION, (float) REVERSE_SOFT_LIMIT_VALUE.getRotations());
		mainSparkMaxWrapper.enableSoftLimit(SOFT_LIMIT_DIRECTION, true);
		mainSparkMaxWrapper.getEncoder().setPositionConversionFactor(ElevatorConstants.GEAR_RATIO);
		mainSparkMaxWrapper.getEncoder().setVelocityConversionFactor(ElevatorConstants.GEAR_RATIO);
		mainSparkMaxWrapper.getPIDController().setP(1);
		mainSparkMaxWrapper.getPIDController().setI(0);
		mainSparkMaxWrapper.getPIDController().setD(0);

		secondarySparkMaxWrapper.setSoftLimit(SOFT_LIMIT_DIRECTION, (float) REVERSE_SOFT_LIMIT_VALUE.getRotations());
		secondarySparkMaxWrapper.enableSoftLimit(SOFT_LIMIT_DIRECTION, true);

		Supplier<Double> motorsVoltage = mainSparkMaxWrapper::getVoltage;
		SuppliedDoubleSignal motorsVoltageSignal = new SuppliedDoubleSignal("main motor voltage", motorsVoltage);

		secondarySparkMaxWrapper.follow(mainSparkMaxWrapper);

		BooleanSupplier atLimitSwitch = () -> mainSparkMaxWrapper.getReverseLimitSwitch(REVERSE_LIMIT_SWITCH_TYPE).isPressed();
		mainSparkMaxWrapper.getReverseLimitSwitch(REVERSE_LIMIT_SWITCH_TYPE).enableLimitSwitch(true);
		IDigitalInput limitSwitch = new SuppliedDigitalInput(atLimitSwitch, DEBOUNCE_TYPE, DEBOUNCE_TIME_SECONDS);

		SparkMaxAngleRequest angleRequest = new SparkMaxAngleRequest(
			Rotation2d.fromRotations(0),
			SparkMaxAngleRequest.SparkAngleRequestType.POSITION,
			ElevatorConstants.ELEVATOR_PID_SLOT,
			RealElevatorConstants::feedforwardCalculation
		);

		return new ElevatorStuff(logPath, mainMotor, motorsVoltageSignal, mainMotorPositionSignal, angleRequest, limitSwitch);
	}

}
