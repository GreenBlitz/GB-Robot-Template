package frc.robot.subsystems.swerve.modules;

import org.littletonrobotics.junction.Logger;

public class ModuleInputsContainer {

	private final ModuleInputsAutoLogged moduleInputs = new ModuleInputsAutoLogged();

	public void processInputs(String logPath) {
		Logger.processInputs(logPath, moduleInputs);
	}

	public ModuleInputsAutoLogged getModuleInputs() {
		return moduleInputs;
	}

}
