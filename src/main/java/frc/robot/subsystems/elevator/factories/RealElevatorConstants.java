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
import frc.robot.hardware.request.cansparkmax.SparkMaxDoubleRequest;
import frc.robot.hardware.signal.supplied.SuppliedAngleSignal;
import frc.robot.hardware.signal.supplied.SuppliedDoubleSignal;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorMotorStuff;
import frc.robot.subsystems.elevator.ElevatorStuff;
import frc.utils.AngleUnit;

import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

public class RealElevatorConstants {

	private static final double DEBOUNCE_TIME_SECONDS = 0.05;

	private static final Debouncer.DebounceType DEBOUNCE_TYPE = Debouncer.DebounceType.kBoth;

	private static final int ELEVATOR_PID_SLOT = 0;

	private static final SparkLimitSwitch.Type REVERSE_LIMIT_SWITCH_TYPE = SparkLimitSwitch.Type.kNormallyOpen;

	private static final ElevatorFeedforward FEEDFORWARD_CALCULATOR = new ElevatorFeedforward(0, 0, 0, 0);

	private static final CANSparkBase.IdleMode IDLE_MODE = CANSparkBase.IdleMode.kCoast;

	private static final int CURRENT_LIMIT = 40;

	private static final Function<
		Rotation2d,
		Double> FEEDFORWARD_FUNCTION = velocity -> RealElevatorConstants.FEEDFORWARD_CALCULATOR.calculate(velocity.getRadians());

	private static void configureMotor(SparkMaxWrapper sparkMaxWrapper, boolean inverted) {
		sparkMaxWrapper.setSoftLimit(
			CANSparkBase.SoftLimitDirection.kReverse,
			(float) Elevator.metersToMotorRotations(ElevatorConstants.REVERSE_SOFT_LIMIT_VALUE_METERS).getRotations()
		);
		sparkMaxWrapper.enableSoftLimit(CANSparkBase.SoftLimitDirection.kReverse, true);
		sparkMaxWrapper.setSoftLimit(
			CANSparkBase.SoftLimitDirection.kForward,
			(float) Elevator.metersToMotorRotations(ElevatorConstants.FORWARD_SOFT_LIMIT_VALUE_METERS).getRotations()
		);
		sparkMaxWrapper.enableSoftLimit(CANSparkBase.SoftLimitDirection.kForward, true);

		sparkMaxWrapper.setIdleMode(IDLE_MODE);
		sparkMaxWrapper.setInverted(inverted);
		sparkMaxWrapper.setSmartCurrentLimit(CURRENT_LIMIT);
		sparkMaxWrapper.getEncoder().setPositionConversionFactor(ElevatorConstants.GEAR_RATIO);
		sparkMaxWrapper.getEncoder().setVelocityConversionFactor(ElevatorConstants.GEAR_RATIO);
		sparkMaxWrapper.getPIDController().setP(1);
		sparkMaxWrapper.getPIDController().setI(0);
		sparkMaxWrapper.getPIDController().setD(0.2);
	}

	public static ElevatorMotorStuff generateMotorStuff(String logPath, SparkMaxWrapper sparkMaxWrapper, boolean inverted) {
		configureMotor(sparkMaxWrapper, inverted);

		ControllableMotor motor = new BrushlessSparkMAXMotor(logPath, sparkMaxWrapper, new SysIdRoutine.Config());

		SuppliedAngleSignal positionSignal = new SuppliedAngleSignal("position", sparkMaxWrapper.getEncoder()::getPosition, AngleUnit.ROTATIONS);

		Supplier<Double> motorVoltage = sparkMaxWrapper::getVoltage;
		SuppliedDoubleSignal voltageSignal = new SuppliedDoubleSignal("voltage", motorVoltage);

		return new ElevatorMotorStuff(motor, voltageSignal, positionSignal);
	}

	public static ElevatorStuff generateElevatorStuff(String logPath) {
		SparkMaxWrapper motorWrapper = new SparkMaxWrapper(IDs.CANSparkMAXIDs.ELEVATOR);

		ElevatorMotorStuff motorStuff = generateMotorStuff(logPath + "motor/", motorWrapper, true);

		BooleanSupplier atLimitSwitch = () -> motorWrapper.getReverseLimitSwitch(REVERSE_LIMIT_SWITCH_TYPE).isPressed();
		motorWrapper.getReverseLimitSwitch(REVERSE_LIMIT_SWITCH_TYPE).enableLimitSwitch(true);
		IDigitalInput limitSwitch = new SuppliedDigitalInput(atLimitSwitch, DEBOUNCE_TYPE, DEBOUNCE_TIME_SECONDS);

		SparkMaxAngleRequest positionRequest = new SparkMaxAngleRequest(
			Rotation2d.fromRotations(0),
			SparkMaxAngleRequest.SparkAngleRequestType.POSITION,
			ELEVATOR_PID_SLOT,
			FEEDFORWARD_FUNCTION
		);

		SparkMaxDoubleRequest voltageRequest = new SparkMaxDoubleRequest(
			0,
			SparkMaxDoubleRequest.SparkDoubleRequestType.VOLTAGE,
			ELEVATOR_PID_SLOT
		);

		return new ElevatorStuff(logPath, positionRequest, voltageRequest, limitSwitch, motorStuff);
	}

}
