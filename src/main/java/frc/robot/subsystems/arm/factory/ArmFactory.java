package frc.robot.subsystems.arm.factory;

import frc.robot.Robot;
import frc.robot.subsystems.arm.Arm;

public class ArmFactory {

	public static Arm create(String logPath) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> TalonFXArmBuilder.build(logPath);

			case SIMULATION -> null;
		};
	}

}
