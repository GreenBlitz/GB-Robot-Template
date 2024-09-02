package frc.robot.subsystems.swerve.factories.modules.drive;

import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveName;
import frc.robot.subsystems.swerve.modules.ModuleConstants;
import frc.robot.subsystems.swerve.modules.ModuleUtils;
import frc.robot.subsystems.swerve.modules.drive.IDrive;
import frc.robot.subsystems.swerve.modules.drive.simulation.SimulationDrive;
import frc.robot.subsystems.swerve.modules.drive.talonfx.TalonFXDrive;

public class DriveFactory {

	public static IDrive create(SwerveName swerveName, ModuleUtils.ModulePosition modulePosition) {
		return switch (swerveName) {
			case SWERVE -> createSwerveDrive(modulePosition);
		};
	}

	private static IDrive createSwerveDrive(ModuleUtils.ModulePosition modulePosition) {
		String logPathPrefix = SwerveName.SWERVE.getLogPath() + ModuleConstants.LOG_PATH_ADDITION;
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> switch (modulePosition) {
				case FRONT_LEFT -> new TalonFXDrive(DriveRealConstants.FRONT_LEFT_CONSTANTS(logPathPrefix));
				case FRONT_RIGHT -> new TalonFXDrive(DriveRealConstants.FRONT_RIGHT_CONSTANTS(logPathPrefix));
				case BACK_LEFT -> new TalonFXDrive(DriveRealConstants.BACK_LEFT_CONSTANTS(logPathPrefix));
				case BACK_RIGHT -> new TalonFXDrive(DriveRealConstants.BACK_RIGHT_CONSTANTS(logPathPrefix));
			};
			case SIMULATION -> new SimulationDrive(DriveSimulationConstants.getDriveConstants());
		};
	}

}
