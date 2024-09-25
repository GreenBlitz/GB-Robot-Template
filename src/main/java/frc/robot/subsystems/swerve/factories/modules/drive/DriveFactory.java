package frc.robot.subsystems.swerve.factories.modules.drive;

import frc.robot.Robot;
import frc.robot.constants.IDs;
import frc.robot.subsystems.swerve.SwerveName;
import frc.robot.subsystems.swerve.modules.ModuleConstants;
import frc.robot.subsystems.swerve.modules.ModuleUtils;
import frc.robot.subsystems.swerve.modules.stuffs.DriveStuff;

public class DriveFactory {

	private static DriveStuff createSwerveDrive(String logPath, ModuleUtils.ModulePosition modulePosition) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> switch (modulePosition) {
				case FRONT_LEFT -> DriveRealConstants.generateDriveStuff(logPath, IDs.TalonFXIDs.FRONT_LEFT_DRIVE_MOTOR, false);
				case FRONT_RIGHT -> DriveRealConstants.generateDriveStuff(logPath, IDs.TalonFXIDs.FRONT_RIGHT_DRIVE_MOTOR, true);
				case BACK_LEFT -> DriveRealConstants.generateDriveStuff(logPath, IDs.TalonFXIDs.BACK_LEFT_DRIVE_MOTOR, false);
				case BACK_RIGHT -> DriveRealConstants.generateDriveStuff(logPath, IDs.TalonFXIDs.BACK_RIGHT_DRIVE_MOTOR, false);
			};
			case SIMULATION -> null;//TODO
		};
	}

	public static DriveStuff create(SwerveName swerveName, ModuleUtils.ModulePosition modulePosition) {
		String logPath = SwerveName.SWERVE.getLogPath() + ModuleConstants.LOG_PATH_ADDITION + modulePosition + "/Drive/";
		return switch (swerveName) {
			case SWERVE -> createSwerveDrive(logPath, modulePosition);
		};
	}

}
