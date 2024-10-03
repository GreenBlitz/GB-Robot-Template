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

	private static double kS = 3;
	private static double kV = 3;
	private static SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV);
	private static Function<Rotation2d, Double> feedForwardCalculator = velocity -> feedforward.calculate(velocity.getRotations());

	private static SysIdRoutine.Config generateSysidConfig() {
		return new SysIdRoutine.Config(
			Volts.of(1).per(Seconds.of(1)),
			Volts.of(7),
			Seconds.of(10),
			(state) -> SignalLogger.writeString("state", state.toString())
		);
	}

	public static FlywheelComponents generateFlywheelComponents(String logPath, boolean isInverted, SparkMaxDeviceID deviceID) {
		SparkMaxWrapper sparkMaxWrapper = new SparkMaxWrapper(deviceID);

		sparkMaxWrapper.setInverted(isInverted);
		sparkMaxWrapper.setSmartCurrentLimit(40);

		SysIdRoutine.Config config = generateSysidConfig();

		BrushlessSparkMAXMotor motor = new BrushlessSparkMAXMotor(logPath, sparkMaxWrapper, config);

		SuppliedDoubleSignal voltageSignal = new SuppliedDoubleSignal("voltage", sparkMaxWrapper::getVoltage);

		SuppliedAngleSignal velocitySignal = new SuppliedAngleSignal("velocity", sparkMaxWrapper.getEncoder()::getVelocity, AngleUnit.ROTATIONS);

		sparkMaxWrapper.getPIDController().setP(5);
		sparkMaxWrapper.getPIDController().setI(0);
		sparkMaxWrapper.getPIDController().setD(0);

		SparkMaxAngleRequest velocityRequest = new SparkMaxAngleRequest(
			Rotation2d.fromRotations(0),
			SparkMaxAngleRequest.SparkAngleRequestType.VELOCITY,
			0,
			feedForwardCalculator
		);

		SparkMaxDoubleRequest voltageRequest = new SparkMaxDoubleRequest(0, SparkMaxDoubleRequest.SparkDoubleRequestType.VOLTAGE, 0);

		return new FlywheelComponents(logPath, motor, isInverted, voltageSignal, velocitySignal, velocityRequest, voltageRequest);
	}

}
