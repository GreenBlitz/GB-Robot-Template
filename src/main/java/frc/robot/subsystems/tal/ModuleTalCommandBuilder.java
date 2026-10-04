package frc.robot.subsystems.tal;

import edu.wpi.first.wpilibj2.command.*;

import java.util.function.Supplier;

public class ModuleTalCommandBuilder {

	private final ModuleTal module;

	public ModuleTalCommandBuilder(ModuleTal module) {
		this.module = module;
		this.module.setDefaultCommand(stop());
	}

	public Command driveModuleByJoystick(Supplier<Double> power, Supplier<Double> angle) {
		return new SequentialCommandGroup(steerToAngle(angle), setDrivePower(power));
	}

	public Command stop() {
		return module.asSubsystemCommand(new RunCommand(module::stop), "stop motors");
	}

	protected Command setDrivePower(Supplier<Double> power) {
		return module.asSubsystemCommand(new RunCommand(() -> module.setDrivePower(power.get())), "set the motors power");
	}

	protected Command steerToAngle(Supplier<Double> angle) {
		return module.asSubsystemCommand(new InstantCommand(() -> module.steerToAngle(angle.get())), "steer to an angle");
	}

}
