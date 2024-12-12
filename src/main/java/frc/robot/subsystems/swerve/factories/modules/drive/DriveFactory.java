package frc.robot.subsystems.swerve.factories.modules.drive;

import frc.robot.Robot;
import frc.robot.constants.IDs;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.phoenix6.motor.TalonFXMotor;
import frc.robot.subsystems.swerve.SwerveType;
import frc.robot.subsystems.swerve.module.ModuleConstants;
import frc.robot.subsystems.swerve.module.ModuleUtils;
import frc.robot.subsystems.swerve.module.records.DriveRequests;
import frc.robot.subsystems.swerve.module.records.DriveSignals;

public class DriveFactory {

	private static ControllableMotor createSwerveDrive(String logPath, ModuleUtils.ModulePosition modulePosition) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL, SIMULATION -> switch (modulePosition) {
				case FRONT_LEFT -> TalonFXDriveConstants.generateDrive(logPath, IDs.TalonFXIDs.FRONT_LEFT_DRIVE_MOTOR, false);
				case FRONT_RIGHT -> TalonFXDriveConstants.generateDrive(logPath, IDs.TalonFXIDs.FRONT_RIGHT_DRIVE_MOTOR, true);
				case BACK_LEFT -> TalonFXDriveConstants.generateDrive(logPath, IDs.TalonFXIDs.BACK_LEFT_DRIVE_MOTOR, false);
				case BACK_RIGHT -> TalonFXDriveConstants.generateDrive(logPath, IDs.TalonFXIDs.BACK_RIGHT_DRIVE_MOTOR, false);
			};
		};
	}

	public static ControllableMotor createDrive(SwerveType swerveType, ModuleUtils.ModulePosition modulePosition) {
		String logPath = SwerveType.SWERVE.getLogPath() + ModuleConstants.LOG_PATH_ADDITION + modulePosition + "/Drive/";
		return switch (swerveType) {
			case SWERVE -> createSwerveDrive(logPath, modulePosition);
		};
	}

	private static DriveRequests createDriveRequests() {
		return switch (Robot.ROBOT_TYPE) {
			case REAL, SIMULATION -> TalonFXDriveConstants.generateRequests();
		};
	}

	public static DriveRequests createRequests(SwerveType swerveType) {
		return switch (swerveType) {
			case SWERVE -> createDriveRequests();
		};
	}

	private static DriveSignals createDriveSignals(ControllableMotor steer) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL, SIMULATION -> TalonFXDriveConstants.generateSignals((TalonFXMotor) steer);
		};
	}

	public static DriveSignals createSignals(SwerveType swerveType, ControllableMotor drive) {
		return switch (swerveType) {
			case SWERVE -> createDriveSignals(drive);
		};
	}

}
