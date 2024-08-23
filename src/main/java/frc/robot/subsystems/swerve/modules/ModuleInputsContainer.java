package frc.robot.subsystems.swerve.modules;

import frc.robot.subsystems.swerve.modules.drive.DriveConstants;
import frc.robot.subsystems.swerve.modules.drive.DriveThreadMetersInputsAutoLogged;
import frc.robot.subsystems.swerve.modules.encoder.EncoderConstants;
import frc.robot.subsystems.swerve.modules.encoder.EncoderInputsAutoLogged;
import frc.robot.subsystems.swerve.modules.steer.SteerConstants;
import frc.robot.subsystems.swerve.modules.steer.SteerThreadInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class ModuleInputsContainer {

	private final ModuleInputsAutoLogged moduleInputs = new ModuleInputsAutoLogged();
	private final EncoderInputsAutoLogged encoderInputs = new EncoderInputsAutoLogged();
	private final SteerThreadInputsAutoLogged steerMotorInputs = new SteerThreadInputsAutoLogged();
	private final DriveThreadMetersInputsAutoLogged driveMotorInputs = new DriveThreadMetersInputsAutoLogged();

	public void processInputs(String logPath) {
		Logger.processInputs(logPath, moduleInputs);
		Logger.processInputs(logPath + EncoderConstants.LOG_PATH_ADDITION, encoderInputs);
		Logger.processInputs(logPath + SteerConstants.LOG_PATH_ADDITION, steerMotorInputs);
		Logger.processInputs(logPath + DriveConstants.LOG_PATH_ADDITION, driveMotorInputs);
	}

	public ModuleInputsAutoLogged getModuleInputs() {
		return moduleInputs;
	}

	public EncoderInputsAutoLogged getEncoderInputs() {
		return encoderInputs;
	}

	public SteerThreadInputsAutoLogged getSteerMotorInputs() {
		return steerMotorInputs;
	}

	public DriveThreadMetersInputsAutoLogged getDriveMotorInputs() {
		return driveMotorInputs;
	}

}
