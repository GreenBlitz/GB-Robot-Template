package frc.robot.subsystems.swerve.factories.swerveconstants;

import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveConstants;

public class SwerveConstantsFactory {

	public static SwerveConstants create(String logPath) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> RealSwerveConstants.getSwerveConstants(logPath);
			case SIMULATION -> SimulationSwerveConstants.getSwerveConstants(logPath);
		};
	}

}
