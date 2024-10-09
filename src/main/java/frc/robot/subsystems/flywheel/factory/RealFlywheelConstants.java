package frc.robot.subsystems.flywheel.factory;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hardware.motor.sparkmax.BrushlessSparkMAXMotor;
import frc.robot.hardware.motor.sparkmax.SparkMaxDeviceID;
import frc.robot.hardware.motor.sparkmax.SparkMaxWrapper;
import frc.robot.hardware.request.cansparkmax.SparkMaxAngleRequest;
import frc.robot.hardware.request.cansparkmax.SparkMaxDoubleRequest;
import frc.robot.hardware.signal.supplied.SuppliedAngleSignal;
import frc.robot.hardware.signal.supplied.SuppliedDoubleSignal;
import frc.robot.subsystems.flywheel.FlywheelComponents;
import frc.utils.AngleUnit;

import java.util.function.Function;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;


public class RealFlywheelConstants {

	private static final double kS = 3;

	private static final double kV = 3;

	private static final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV);

	private static final Function<Rotation2d, Double> feedForwardCalculator = velocity -> feedforward.calculate(velocity.getRotations());

	private static final double GEAR_RATIO = 1;

	private static SysIdRoutine.Config generateSysidConfig() {
		return new SysIdRoutine.Config(
			Volts.of(1).per(Seconds.of(1)),
			Volts.of(7),
			Seconds.of(10),
			(state) -> SignalLogger.writeString("state", state.toString())
		);
	}

	private static void configMotor(SparkMaxWrapper sparkMax, boolean isInverted) {
		sparkMax.setInverted(isInverted);
		sparkMax.setSmartCurrentLimit(40);
		sparkMax.getEncoder().setPositionConversionFactor(GEAR_RATIO);
		sparkMax.getEncoder().setVelocityConversionFactor(GEAR_RATIO);

		sparkMax.getPIDController().setP(5);
		sparkMax.getPIDController().setI(0);
		sparkMax.getPIDController().setD(0);
	}

	public static FlywheelComponents generateFlywheelComponents(String logPath, boolean isInverted, SparkMaxDeviceID deviceID) {
		SparkMaxWrapper sparkMaxWrapper = new SparkMaxWrapper(deviceID);

		configMotor(sparkMaxWrapper, isInverted);

		SysIdRoutine.Config config = generateSysidConfig();

		BrushlessSparkMAXMotor motor = new BrushlessSparkMAXMotor(logPath, sparkMaxWrapper, config);

		SuppliedDoubleSignal voltageSignal = new SuppliedDoubleSignal("voltage", sparkMaxWrapper::getVoltage);

		SuppliedAngleSignal velocitySignal = new SuppliedAngleSignal("velocity", sparkMaxWrapper.getEncoder()::getVelocity, AngleUnit.ROTATIONS);

		SparkMaxAngleRequest velocityRequest = new SparkMaxAngleRequest(
			Rotation2d.fromRotations(0),
			SparkMaxAngleRequest.SparkAngleRequestType.VELOCITY,
			0,
			feedForwardCalculator
		);

		SparkMaxDoubleRequest voltageRequest = new SparkMaxDoubleRequest(0, SparkMaxDoubleRequest.SparkDoubleRequestType.VOLTAGE, 0);

		return new FlywheelComponents(logPath, motor, isInverted, voltageSignal, velocitySignal, voltageRequest, velocityRequest);
	}

}
