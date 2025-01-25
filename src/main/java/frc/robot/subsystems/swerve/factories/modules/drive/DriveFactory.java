package frc.robot.subsystems.swerve.factories.modules.drive;

import frc.robot.Robot;
import frc.robot.IDs;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.subsystems.swerve.module.ModuleConstants;
import frc.robot.subsystems.swerve.module.ModuleUtils;
import frc.robot.subsystems.swerve.module.records.DriveRequests;
import frc.robot.subsystems.swerve.module.records.DriveSignals;

public class DriveFactory {

	public static ControllableMotor createDrive(String logPath, ModuleUtils.ModulePosition modulePosition) {
		logPath += ModuleConstants.MODULES_LOG_PATH_ADDITION + "/" + modulePosition + "/Drive";
		return switch (Robot.ROBOT_TYPE) {
			case REAL, SIMULATION -> switch (modulePosition) {
				case FRONT_LEFT -> KrakenX60DriveBuilder.buildDrive(logPath, IDs.TalonFXIDs.FRONT_LEFT_DRIVE_MOTOR, false);
				case FRONT_RIGHT -> KrakenX60DriveBuilder.buildDrive(logPath, IDs.TalonFXIDs.FRONT_RIGHT_DRIVE_MOTOR, true);
				case BACK_LEFT -> KrakenX60DriveBuilder.buildDrive(logPath, IDs.TalonFXIDs.BACK_LEFT_DRIVE_MOTOR, false);
				case BACK_RIGHT -> KrakenX60DriveBuilder.buildDrive(logPath, IDs.TalonFXIDs.BACK_RIGHT_DRIVE_MOTOR, false);
			};
		};
	}

	public static DriveRequests createRequests() {
		return switch (Robot.ROBOT_TYPE) {
			case REAL, SIMULATION -> KrakenX60DriveBuilder.buildRequests();
		};
	}

	public static DriveSignals createSignals(ControllableMotor drive) {
		return switch (Robot.ROBOT_TYPE) {
			case REAL, SIMULATION -> KrakenX60DriveBuilder.buildSignals((TalonFXMotor) drive);
		};
	}

}
