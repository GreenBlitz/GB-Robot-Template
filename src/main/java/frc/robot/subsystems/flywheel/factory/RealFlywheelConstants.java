package frc.robot.subsystems.flywheel.factory;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hardware.motor.sparkmax.BrushlessSparkMAXMotor;
import frc.robot.hardware.motor.sparkmax.SparkMaxDeviceID;
import frc.robot.hardware.motor.sparkmax.SparkMaxWrapper;
import frc.robot.hardware.request.cansparkmax.SparkMaxAngleRequest;
import frc.robot.hardware.signal.supplied.SuppliedAngleSignal;
import frc.robot.hardware.signal.supplied.SuppliedDoubleSignal;
import frc.robot.subsystems.flywheel.FlywheelComponents;
import frc.utils.AngleUnit;

public class RealFlywheelConstants {

	public static FlywheelComponents generateTopFlywheelComponents(String logPath, boolean isInverted, SparkMaxDeviceID deviceID) {
		SparkMaxWrapper sparkMaxWrapper = new SparkMaxWrapper(deviceID);

		sparkMaxWrapper.getEncoder().setInverted(isInverted);
		sparkMaxWrapper.setSmartCurrentLimit(40);

		SysIdRoutine.Config config = new SysIdRoutine.Config();

		BrushlessSparkMAXMotor motor = new BrushlessSparkMAXMotor(logPath, sparkMaxWrapper, config);

		SuppliedDoubleSignal voltageSignal = new SuppliedDoubleSignal("voltage", sparkMaxWrapper::getVoltage);


		SuppliedAngleSignal velocitySignal = new SuppliedAngleSignal("velocity", sparkMaxWrapper.getEncoder()::getVelocity, AngleUnit.ROTATIONS);

		SparkMaxAngleRequest velocityRequest = new SparkMaxAngleRequest(
			Rotation2d.fromRotations(60),
			SparkMaxAngleRequest.SparkAngleRequestType.VELOCITY,
			0
		);

		return new FlywheelComponents(logPath, motor, isInverted, voltageSignal, velocitySignal, velocityRequest);
	}

}
