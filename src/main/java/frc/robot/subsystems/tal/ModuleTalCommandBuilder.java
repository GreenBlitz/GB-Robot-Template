package frc.robot.subsystems.tal;

import edu.wpi.first.wpilibj2.command.*;

import java.util.function.Supplier;

public class ModuleTalCommandBuilder {

	private final ModuleTal module;

	public ModuleTalCommandBuilder(ModuleTal module) {
		this.module = module;
		this.module.setDefaultCommand(stop());
	}

	public Command driveModuleByJoystick(Supplier<Double> angle, Supplier<Double> power) {
		return module
			.asSubsystemCommand(new RunCommand(() -> module.drive(angle.get(), power.get()), module), "drive module by power and angle");
	}

	public Command stop() {
		return module.asSubsystemCommand(new RunCommand(module::stop), "stop motors");
	}

	public Command setDrivePower(Supplier<Double> power) {
		return module.asSubsystemCommand(new RunCommand(() -> module.setDrivePower(power.get()), module), "set the motors power");
	}

	public Command steerToAngle(Supplier<Double> angle) {
		return module.asSubsystemCommand(new RunCommand(() -> module.steerToAngle(angle.get()), module), "steer to an angle");
	}

}
