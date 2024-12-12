package frc.robot.subsystems.swerve.factories.swerveconstants;

import frc.robot.RobotOld;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveType;

public class SwerveConstantsFactory {

	private static SwerveConstants createSwerveConstants() {
		return switch (RobotOld.ROBOT_TYPE) {
			case REAL -> RealSwerveConstants.getSwerveConstants(SwerveType.SWERVE);
			case SIMULATION -> SimulationSwerveConstants.getSwerveConstants(SwerveType.SWERVE);
		};
	}

	public static SwerveConstants create(SwerveType swerveType) {
		return switch (swerveType) {
			case SWERVE -> createSwerveConstants();
		};
	}

}
