package frc.robot.subsystems.factories.arm;

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
