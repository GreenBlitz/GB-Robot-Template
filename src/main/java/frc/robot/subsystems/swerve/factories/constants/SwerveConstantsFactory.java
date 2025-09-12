package frc.robot.subsystems.swerve.factories.constants;

import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveConstants;

public class SwerveConstantsFactory {

	public static SwerveConstants create(String logPath) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> RealSwerveConstants.getSwerveConstants(logPath);
			case REPLAY, SIMULATION -> SimulationSwerveConstants.getSwerveConstants(logPath);
		};
	}

}
