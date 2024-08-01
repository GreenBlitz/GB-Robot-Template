package frc.robot.subsystems.swerve.modules.inputs;

import frc.robot.subsystems.swerve.modules.check.drive.DriveInputsAutoLogged;
import frc.robot.subsystems.swerve.modules.check.encoder.EncoderInputsAutoLogged;
import frc.robot.subsystems.swerve.modules.check.steer.SteerInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class ModuleInputsContainer {

    private final ModuleInputsAutoLogged moduleInputs = new ModuleInputsAutoLogged();
    private final EncoderInputsAutoLogged encoderInputs = new EncoderInputsAutoLogged();
    private final SteerInputsAutoLogged steerMotorInputs = new SteerInputsAutoLogged();
    private final DriveInputsAutoLogged driveMotorInputs = new DriveInputsAutoLogged();

    public void processInputs(String loggingPath){
        Logger.processInputs(loggingPath, moduleInputs);
        Logger.processInputs(loggingPath + "Encoder", encoderInputs);
        Logger.processInputs(loggingPath + "Steer Motor", steerMotorInputs);
        Logger.processInputs(loggingPath + "Drive Motor", driveMotorInputs);
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
