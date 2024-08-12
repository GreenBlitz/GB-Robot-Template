package frc.robot.subsystems.swerve.modules;

import frc.robot.subsystems.swerve.modules.drive.DriveInputsAutoLogged;
import frc.robot.subsystems.swerve.modules.drive.talonfx.TalonFXDriveConstants;
import frc.robot.subsystems.swerve.modules.encoder.EncoderInputsAutoLogged;
import frc.robot.subsystems.swerve.modules.encoder.cancoder.CancoderEncoderConstants;
import frc.robot.subsystems.swerve.modules.steer.SteerInputsAutoLogged;
import frc.robot.subsystems.swerve.modules.steer.talonfx.TalonFXSteerConstants;
import org.littletonrobotics.junction.Logger;

public class ModuleInputsContainer {

	private final ModuleInputsAutoLogged moduleInputs = new ModuleInputsAutoLogged();
	private final EncoderInputsAutoLogged encoderInputs = new EncoderInputsAutoLogged();
	private final SteerInputsAutoLogged steerMotorInputs = new SteerInputsAutoLogged();
	private final DriveInputsAutoLogged driveMotorInputs = new DriveInputsAutoLogged();

	public void processInputs(String logPath) {
		Logger.processInputs(logPath, moduleInputs);
		Logger.processInputs(logPath + CancoderEncoderConstants.LOG_PATH_ADDITION, encoderInputs);
		Logger.processInputs(logPath + TalonFXSteerConstants.LOG_PATH_ADDITION, steerMotorInputs);
		Logger.processInputs(logPath + TalonFXDriveConstants.LOG_PATH_ADDITION, driveMotorInputs);
	}

	public ModuleInputsAutoLogged getModuleInputs() {
		return moduleInputs;
	}

	public EncoderInputsAutoLogged getEncoderInputs() {
		return encoderInputs;
	}

	public SteerInputsAutoLogged getSteerMotorInputs() {
		return steerMotorInputs;
	}

	public DriveInputsAutoLogged getDriveMotorInputs() {
		return driveMotorInputs;
	}

}
