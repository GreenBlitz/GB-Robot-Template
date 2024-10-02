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

	public static FlywheelComponents generateTopFlywheelComponents(String logPath, boolean isMotorInverted, SparkMaxDeviceID ID) {
		SparkMaxWrapper SparkMaxWrapper = new SparkMaxWrapper(ID);

		SparkMaxWrapper.getEncoder().setInverted(isMotorInverted);
		SparkMaxWrapper.setSmartCurrentLimit(40);

		SysIdRoutine.Config Config = new SysIdRoutine.Config();

		BrushlessSparkMAXMotor Motor = new BrushlessSparkMAXMotor(logPath, SparkMaxWrapper, Config);

		SuppliedDoubleSignal MotorVoltageSignal = new SuppliedDoubleSignal("voltage", SparkMaxWrapper::getVoltage);


		SuppliedAngleSignal MotorVelocitySignal = new SuppliedAngleSignal(
			"velocity",
			SparkMaxWrapper.getEncoder()::getVelocity,
			AngleUnit.ROTATIONS
		);

		SparkMaxAngleRequest MotorVelocityRequest = new SparkMaxAngleRequest(
			Rotation2d.fromRotations(60),
			SparkMaxAngleRequest.SparkAngleRequestType.VELOCITY,
			0
		);

		return new FlywheelComponents(logPath, Motor, isMotorInverted, MotorVoltageSignal, MotorVelocitySignal, MotorVelocityRequest);
	}

}
