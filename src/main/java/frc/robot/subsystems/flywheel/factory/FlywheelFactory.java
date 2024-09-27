package frc.robot.subsystems.flywheel.factory;

import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.IDs;
import frc.robot.hardware.motor.phoenix6.TalonFXWrapper;
import frc.robot.hardware.request.phoenix6.Phoenix6AngleRequest;
import frc.robot.hardware.signal.InputSignal;
import frc.robot.hardware.signal.phoenix.Phoenix6SignalBuilder;
import frc.robot.subsystems.flywheel.Flywheel;

import static frc.robot.subsystems.flywheel.factory.RealFlywheelConstants.*;

public class FlywheelFactory {

	private static Flywheel getRealFlywheel(String logPath) {
		TalonFXWrapper rightMotorWrapper = new TalonFXWrapper(IDs.TalonFXs.RIGHT_FLYWHEEL);
		rightMotorWrapper.applyConfiguration(CONFIGURATION);

		TalonFXWrapper leftMotorWrapper = new TalonFXWrapper(IDs.TalonFXs.LEFT_FLYWHEEL);
		leftMotorWrapper.applyConfiguration(CONFIGURATION);

		return new Flywheel(
			logPath,
			RealFlywheelConstants.getTalonFXMotor(logPath + "right/", rightMotorWrapper),
			RealFlywheelConstants.getTalonFXMotor(logPath + "left/", leftMotorWrapper),
			RealFlywheelConstants.generateSignal(rightMotorWrapper.getVelocity()),
			RealFlywheelConstants.generateSignal(leftMotorWrapper.getVelocity()),
			new Phoenix6AngleRequest(new VelocityVoltage(0)),
			new Phoenix6AngleRequest(new VelocityVoltage(0)),
			new InputSignal[] {
				Phoenix6SignalBuilder.generatePhoenix6Signal(rightMotorWrapper.getVelocity(), GlobalConstants.ROBORIO_CANBUS_UPDATE_FREQUENCY),
				Phoenix6SignalBuilder
					.generatePhoenix6Signal(rightMotorWrapper.getMotorVoltage(), GlobalConstants.ROBORIO_CANBUS_UPDATE_FREQUENCY),
				Phoenix6SignalBuilder
					.generatePhoenix6Signal(rightMotorWrapper.getStatorCurrent(), GlobalConstants.ROBORIO_CANBUS_UPDATE_FREQUENCY),},
			new InputSignal[] {
				Phoenix6SignalBuilder.generatePhoenix6Signal(leftMotorWrapper.getVelocity(), GlobalConstants.ROBORIO_CANBUS_UPDATE_FREQUENCY),
				Phoenix6SignalBuilder
					.generatePhoenix6Signal(leftMotorWrapper.getMotorVoltage(), GlobalConstants.ROBORIO_CANBUS_UPDATE_FREQUENCY),
				Phoenix6SignalBuilder
					.generatePhoenix6Signal(leftMotorWrapper.getStatorCurrent(), GlobalConstants.ROBORIO_CANBUS_UPDATE_FREQUENCY),}
		);
	}

	public static Flywheel generateFlywheel(String logPath) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> getRealFlywheel(logPath);
			default -> null;
		};
	}

}
