package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.controls.VelocityVoltage;
import frc.robot.RobotType;
import frc.robot.constants.IDs;
import frc.robot.hardware.motor.phoenix6.TalonFXMotor;
import frc.robot.hardware.motor.phoenix6.TalonFXWrapper;
import frc.robot.hardware.request.phoenix6.Phoenix6AngleRequest;
import frc.robot.hardware.signal.InputSignal;
import frc.robot.hardware.signal.phoenix.Phoenix6SignalBuilder;
import frc.utils.AngleUnit;

import static frc.robot.subsystems.flywheel.FlywheelConstants.*;

public class FlywheelFactory {

	public static Flywheel getRealFlywheel(String logPath) {
		TalonFXWrapper rightMotorWrapper = new TalonFXWrapper(IDs.TalonFXs.RIGHT_FLYWHEEL);
		rightMotorWrapper.applyConfiguration(CONFIGURATION);

		TalonFXWrapper leftMotorWrapper = new TalonFXWrapper(IDs.TalonFXs.LEFT_FLYWHEEL);
		rightMotorWrapper.applyConfiguration(CONFIGURATION);

		return new Flywheel(
			logPath,
			new TalonFXMotor(logPath + "right/", rightMotorWrapper, SYSID_CONFIG),
			new TalonFXMotor(logPath + "left/", leftMotorWrapper, SYSID_CONFIG),
			Phoenix6SignalBuilder.generatePhoenix6Signal(rightMotorWrapper.getVelocity(), REFRESH_HERTZ, AngleUnit.ROTATIONS),
			Phoenix6SignalBuilder.generatePhoenix6Signal(leftMotorWrapper.getVelocity(), REFRESH_HERTZ, AngleUnit.ROTATIONS),
			new Phoenix6AngleRequest(new VelocityVoltage(0)),
			new Phoenix6AngleRequest(new VelocityVoltage(0)),
			new InputSignal[] {},
			new InputSignal[] {}
		);
	}

    public static Flywheel generateFlywheel(RobotType type, String logPath){
        return switch (type){
            case REAL -> getRealFlywheel(logPath);
            default -> null;
        };
    }

}
