package frc.robot.subsystems.endEffector.factory;

import frc.robot.Robot;
import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.endEffector.EndEffectorConstants;

public class EndEffectorFactory {

	public static EndEffector create() {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> RealEndEffectorConstants.generateReal(EndEffectorConstants.LOG_PATH, EndEffectorConstants.MOTOR_LOG_PATH);
			case SIMULATION -> RealEndEffectorConstants.generateSim(EndEffectorConstants.LOG_PATH, EndEffectorConstants.MOTOR_LOG_PATH);
		};
	}

}

