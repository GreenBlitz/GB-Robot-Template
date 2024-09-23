package frc.robot.subsystems.swerve.modules;

import frc.robot.subsystems.swerve.modules.drive.DriveConstants;
import frc.robot.subsystems.swerve.modules.drive.DriveInputsAutoLogged;
import frc.robot.subsystems.swerve.modules.steer.SteerConstants;
import frc.robot.subsystems.swerve.modules.steer.SteerInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class ModuleInputsContainer {

	private final ModuleInputsAutoLogged moduleInputs = new ModuleInputsAutoLogged();
	private final SteerInputsAutoLogged steerMotorInputs = new SteerInputsAutoLogged();
	private final DriveInputsAutoLogged driveMotorInputs = new DriveInputsAutoLogged();

	public void processInputs(String logPath) {
		Logger.processInputs(logPath, moduleInputs);
		Logger.processInputs(logPath + SteerConstants.LOG_PATH_ADDITION, steerMotorInputs);
		Logger.processInputs(logPath + DriveConstants.LOG_PATH_ADDITION, driveMotorInputs);
	}

	public ModuleInputsAutoLogged getModuleInputs() {
		return moduleInputs;
	}

	public SteerInputsAutoLogged getSteerMotorInputs() {
		return steerMotorInputs;
	}

	public DriveInputsAutoLogged getDriveMotorInputs() {
		return driveMotorInputs;
	}

}
