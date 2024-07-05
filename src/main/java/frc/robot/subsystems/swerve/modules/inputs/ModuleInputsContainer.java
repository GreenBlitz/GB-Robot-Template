package frc.robot.subsystems.swerve.modules.inputs;

import org.littletonrobotics.junction.Logger;

public class ModuleInputsContainer {

    private final ModuleInputsAutoLogged moduleInputs = new ModuleInputsAutoLogged();
    private final EncoderInputsAutoLogged encoderInputs = new EncoderInputsAutoLogged();
    private final SteerMotorInputsAutoLogged steerMotorInputs = new SteerMotorInputsAutoLogged();
    private final DriveMotorInputsAutoLogged driveMotorInputs = new DriveMotorInputsAutoLogged();

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

    public SteerMotorInputsAutoLogged getSteerMotorInputs() {
        return steerMotorInputs;
    }

    public DriveMotorInputsAutoLogged getDriveMotorInputs() {
        return driveMotorInputs;
    }

}
