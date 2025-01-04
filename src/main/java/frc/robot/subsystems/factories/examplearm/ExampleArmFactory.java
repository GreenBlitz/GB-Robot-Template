package frc.robot.subsystems.factories.examplearm;


import frc.robot.Robot;
import frc.robot.subsystems.arm.ExampleArm;

public class ExampleArmFactory {

	public static ExampleArm create(String logPath) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> ExampleTalonFXArmBuilder.build(logPath);

			case SIMULATION -> null;
		};
	}

}
