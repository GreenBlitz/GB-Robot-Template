package frc.robot.subsystems.flywheel.factory;

import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.IDs;
import frc.robot.hardware.motor.sparkmax.BrushlessSparkMAXMotor;
import frc.robot.hardware.motor.sparkmax.SparkMaxWrapper;
import frc.robot.hardware.request.phoenix6.Phoenix6AngleRequest;
import frc.robot.hardware.signal.cansparkmax.SparkMaxAngleSignal;
import frc.robot.hardware.signal.cansparkmax.SparkMaxDoubleSignal;
import frc.robot.subsystems.flywheel.BottomFlywheelComponents;
import frc.robot.subsystems.flywheel.TopFlywheelComponents;
import frc.utils.AngleUnit;

import java.util.function.Supplier;

public class RealFlywheelConstants {

	public static TopFlywheelComponents generateTopFlywheelComponents(String logpath, boolean isMotorInverted) {
		SparkMaxWrapper topSparkMaxWrapper = new SparkMaxWrapper(IDs.CANSparkMaxIDs.TOP_FLYWHEEL_MOTOR);

		topSparkMaxWrapper.getEncoder().setInverted(isMotorInverted);

		SysIdRoutine.Config topConfig = new SysIdRoutine.Config();

		BrushlessSparkMAXMotor topMotor = new BrushlessSparkMAXMotor(logpath, topSparkMaxWrapper, topConfig);

		Supplier<Double> topMotorVoltageSupplier = () -> topSparkMaxWrapper.getBusVoltage() * topSparkMaxWrapper.getAppliedOutput();
		SparkMaxDoubleSignal topMotorVoltageSignal = new SparkMaxDoubleSignal("Top flywheel motor voltage Signal", topMotorVoltageSupplier);


		Supplier<Double> topMotorVelocitySupplier = () -> topSparkMaxWrapper.getEncoder().getVelocity();
		SparkMaxAngleSignal topMotorVelocitySignal = new SparkMaxAngleSignal(
			"Top flywheel motor velocity Signal",
			topMotorVelocitySupplier,
			AngleUnit.ROTATIONS
		);

		Phoenix6AngleRequest topMotorVelocityRequest = new Phoenix6AngleRequest(new VelocityVoltage(0));

		return new TopFlywheelComponents(
			logpath,
			topMotor,
			isMotorInverted,
			topMotorVoltageSignal,
			topMotorVelocitySignal,
			topMotorVelocityRequest
		);
	}


	public static BottomFlywheelComponents generateBottomFlywheelComponents(String logpath, boolean isMotorInverted) {
		SparkMaxWrapper bottomSparkMaxWrapper = new SparkMaxWrapper(IDs.CANSparkMaxIDs.BOTTOM_FLYWHEEL_MOTOR);

		bottomSparkMaxWrapper.getEncoder().setInverted(isMotorInverted);

		SysIdRoutine.Config bottomConfig = new SysIdRoutine.Config();

		BrushlessSparkMAXMotor bottomMotor = new BrushlessSparkMAXMotor(logpath, bottomSparkMaxWrapper, bottomConfig);


		Supplier<Double> bottomMotorVoltageSupplier = () -> bottomSparkMaxWrapper.getBusVoltage() * bottomSparkMaxWrapper.getAppliedOutput();
		SparkMaxDoubleSignal bottomMotorVoltageSignal = new SparkMaxDoubleSignal(
				"Bottom flywheel motor voltage Signal",
				bottomMotorVoltageSupplier
		);


		Supplier<Double> bottomMotorVelocitySupplier = () -> bottomSparkMaxWrapper.getEncoder().getVelocity();
		SparkMaxAngleSignal bottomMotorVelocitySignal = new SparkMaxAngleSignal(
				"Bottom flywheel motor velocity Signal",
				bottomMotorVelocitySupplier,
				AngleUnit.ROTATIONS
		);

		Phoenix6AngleRequest bottomMotorVelocityRequest = new Phoenix6AngleRequest(new VelocityVoltage(0));

		return new BottomFlywheelComponents(
				logpath,
				bottomMotor,
				isMotorInverted,
				bottomMotorVoltageSignal,
				bottomMotorVelocitySignal,
				bottomMotorVelocityRequest
		);
	}

}
