package frc.robot.subsystems.swerve.factories.modules.drive;

import frc.robot.IDs;
import frc.robot.Robot;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.phoenix6.motors.TalonFXMotor;
import frc.robot.subsystems.swerve.maplewrappers.MapleControllableModuleMotor;
import frc.robot.subsystems.swerve.module.ModuleConstants;
import frc.robot.subsystems.swerve.module.ModuleUtil;
import frc.robot.subsystems.swerve.module.records.DriveRequests;
import frc.robot.subsystems.swerve.module.records.DriveSignals;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

public class DriveFactory {

	public static ControllableMotor createDrive(
		String logPath,
		ModuleUtil.ModulePosition modulePosition,
		SwerveDriveSimulation swerveDriveSimulation
	) {
		logPath += ModuleConstants.MODULES_LOG_PATH_ADDITION + "/" + modulePosition + "/Drive";
		if (Robot.ROBOT_TYPE.isSimulation()) {
			return switch (modulePosition) {
				case FRONT_LEFT -> MapleDriveBuilder.buildDrive(logPath, swerveDriveSimulation.getModules()[0]);
				case FRONT_RIGHT -> MapleDriveBuilder.buildDrive(logPath, swerveDriveSimulation.getModules()[1]);
				case BACK_LEFT -> MapleDriveBuilder.buildDrive(logPath, swerveDriveSimulation.getModules()[2]);
				case BACK_RIGHT -> MapleDriveBuilder.buildDrive(logPath, swerveDriveSimulation.getModules()[3]);
			};
		}
		return switch (modulePosition) {
			case FRONT_LEFT -> KrakenX60DriveBuilder.buildDrive(logPath, IDs.TalonFXIDs.SWERVE_FRONT_LEFT_DRIVE, true);
			case FRONT_RIGHT -> KrakenX60DriveBuilder.buildDrive(logPath, IDs.TalonFXIDs.SWERVE_FRONT_RIGHT_DRIVE, true);
			case BACK_LEFT -> KrakenX60DriveBuilder.buildDrive(logPath, IDs.TalonFXIDs.SWERVE_BACK_LEFT_DRIVE, true);
			case BACK_RIGHT -> KrakenX60DriveBuilder.buildDrive(logPath, IDs.TalonFXIDs.SWERVE_BACK_RIGHT_DRIVE, true);
		};
	}

	public static DriveRequests createRequests() {
		return Robot.ROBOT_TYPE.isSimulation() ? MapleDriveBuilder.buildRequests() : KrakenX60DriveBuilder.buildRequests();
	}

	public static DriveSignals createSignals(ControllableMotor drive) {
		return Robot.ROBOT_TYPE.isSimulation()
			? MapleDriveBuilder.buildSignals((MapleControllableModuleMotor) drive)
			: KrakenX60DriveBuilder.buildSignals((TalonFXMotor) drive);
	}

}
