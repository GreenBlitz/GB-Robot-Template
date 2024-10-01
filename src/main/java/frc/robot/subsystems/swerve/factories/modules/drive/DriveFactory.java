package frc.robot.subsystems.swerve.factories.modules.drive;

import frc.robot.Robot;
import frc.robot.constants.IDs;
import frc.robot.subsystems.swerve.SwerveType;
import frc.robot.subsystems.swerve.module.ModuleConstants;
import frc.robot.subsystems.swerve.module.ModuleUtils;
import frc.robot.subsystems.swerve.module.stuffs.DriveStuff;

public class DriveFactory {

	private static DriveStuff createSwerveDrive(String logPath, ModuleUtils.ModulePosition modulePosition) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL -> switch (modulePosition) {
				case FRONT_LEFT -> DriveRealConstants.generateDriveStuff(logPath, IDs.TalonFXIDs.FRONT_LEFT_DRIVE_MOTOR, false);
				case FRONT_RIGHT -> DriveRealConstants.generateDriveStuff(logPath, IDs.TalonFXIDs.FRONT_RIGHT_DRIVE_MOTOR, true);
				case BACK_LEFT -> DriveRealConstants.generateDriveStuff(logPath, IDs.TalonFXIDs.BACK_LEFT_DRIVE_MOTOR, false);
				case BACK_RIGHT -> DriveRealConstants.generateDriveStuff(logPath, IDs.TalonFXIDs.BACK_RIGHT_DRIVE_MOTOR, false);
			};
			case SIMULATION -> null;// TODO
		};
	}

	public static DriveStuff create(SwerveType swerveType, ModuleUtils.ModulePosition modulePosition) {
		String logPath = SwerveType.SWERVE.getLogPath() + ModuleConstants.LOG_PATH_ADDITION + modulePosition + "/Drive/";
		return switch (swerveType) {
			case SWERVE -> createSwerveDrive(logPath, modulePosition);
		};
	}

}
