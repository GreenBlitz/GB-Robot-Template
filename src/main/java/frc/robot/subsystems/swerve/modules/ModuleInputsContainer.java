package frc.robot.subsystems.swerve.modules;

import frc.robot.hardware.MotorInputs;
import frc.robot.subsystems.swerve.modules.drive.DriveConstants;
import frc.robot.subsystems.swerve.modules.drive.DriveInputsAutoLogged;
import frc.robot.subsystems.swerve.modules.encoder.EncoderConstants;
import frc.robot.subsystems.swerve.modules.encoder.EncoderInputsAutoLogged;
import frc.robot.subsystems.swerve.modules.steer.SteerConstants;
import frc.robot.subsystems.swerve.modules.steer.SteerInputs;
import frc.robot.subsystems.swerve.modules.steer.SteerInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class ModuleInputsContainer {

	private final ModuleInputsAutoLogged moduleInputs = new ModuleInputsAutoLogged();
	private final EncoderInputsAutoLogged encoderInputs = new EncoderInputsAutoLogged();
	private final MotorInputs steerMotorInputs = new MotorInputs();
	private final SteerInputs steerInputs = new SteerInputs();
	private final DriveInputsAutoLogged driveMotorInputs = new DriveInputsAutoLogged();

	public void processInputs(String logPath) {
		Logger.processInputs(logPath, moduleInputs);
		Logger.processInputs(logPath + EncoderConstants.LOG_PATH_ADDITION, encoderInputs);
		Logger.processInputs(logPath + SteerConstants.LOG_PATH_ADDITION, steerMotorInputs);
		Logger.processInputs(logPath + SteerConstants.LOG_PATH_ADDITION, steerInputs);
		Logger.processInputs(logPath + DriveConstants.LOG_PATH_ADDITION, driveMotorInputs);
	}

	public ModuleInputsAutoLogged getModuleInputs() {
		return moduleInputs;
	}

	public EncoderInputsAutoLogged getEncoderInputs() {
		return encoderInputs;
	}

	public MotorInputs getSteerMotorInputs() {
		return steerMotorInputs;
	}
	public SteerInputsAutoLogged getSteerInputs() {
		return steerInputs;
	}

	public DriveInputsAutoLogged getDriveMotorInputs() {
		return driveMotorInputs;
	}

}
