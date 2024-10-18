package frc.robot.subsystems.flywheel.factory;

import com.revrobotics.CANSparkBase;
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
import frc.robot.subsystems.flywheel.FlywheelStuff;
import frc.utils.AngleUnit;
import org.littletonrobotics.junction.Logger;

import java.util.function.Function;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

public class RealFlywheelConstants {

	private static final double kS = 0.37804;

	private static final double kV = 0.13081;

	private static final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV);

	private static final Function<Rotation2d, Double> feedForwardCalculator = velocity -> feedforward.calculate(velocity.getRotations());

	private static final double GEAR_RATIO = 1;

	private static SysIdRoutine.Config generateSysidConfig() {
		return new SysIdRoutine.Config(
			Volts.of(1).per(Seconds.of(1)),
			Volts.of(7),
			Seconds.of(10),
			(state) -> Logger.recordOutput("state", state.toString())
		);
	}

	private static void configMotor(SparkMaxWrapper sparkMax, boolean isInverted) {
		sparkMax.setInverted(isInverted);
		sparkMax.setSmartCurrentLimit(40);
		sparkMax.getEncoder().setPositionConversionFactor(GEAR_RATIO);
		sparkMax.getEncoder().setVelocityConversionFactor(GEAR_RATIO);
		sparkMax.setIdleMode(CANSparkBase.IdleMode.kCoast);

		sparkMax.getPIDController().setP(0);
		sparkMax.getPIDController().setI(0);
		sparkMax.getPIDController().setD(0);
	}

	public static FlywheelStuff generateFlywheelStuff(String logPath, boolean isInverted, SparkMaxDeviceID deviceID) {
		SparkMaxWrapper sparkMaxWrapper = new SparkMaxWrapper(deviceID);

		configMotor(sparkMaxWrapper, isInverted);

		SysIdRoutine.Config config = generateSysidConfig();

		BrushlessSparkMAXMotor motor = new BrushlessSparkMAXMotor(logPath, sparkMaxWrapper, config);

		SuppliedDoubleSignal voltageSignal = new SuppliedDoubleSignal("voltage", sparkMaxWrapper::getVoltage);

		SuppliedAngleSignal velocitySignal = new SuppliedAngleSignal(
			"velocity",
			() -> sparkMaxWrapper.getEncoder().getVelocity() / 60.0,
			AngleUnit.ROTATIONS
		);

		SparkMaxAngleRequest velocityRequest = new SparkMaxAngleRequest(
			Rotation2d.fromRotations(0),
			SparkMaxAngleRequest.SparkAngleRequestType.VELOCITY,
			0,
			feedForwardCalculator
		);

		SparkMaxDoubleRequest voltageRequest = new SparkMaxDoubleRequest(0, SparkMaxDoubleRequest.SparkDoubleRequestType.VOLTAGE, 0);

		return new FlywheelStuff(logPath, motor, isInverted, voltageSignal, velocitySignal, voltageRequest, velocityRequest);
	}

}
