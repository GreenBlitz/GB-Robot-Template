package frc.robot.subsystems.swerve.factories.swerveconstants;

import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveName;

public class SwerveConstantsFactory {

	public static SwerveConstants create(SwerveName swerveName) {
		return switch (swerveName) {
			case SWERVE -> createSwerveConstants();
		};
	}

	private static SwerveConstants createSwerveConstants() {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> RealSwerveConstants.getSwerveConstants(SwerveName.SWERVE);
			case SIMULATION -> SimulationSwerveConstants.getSwerveConstants(SwerveName.SWERVE);
		};
	}

}
