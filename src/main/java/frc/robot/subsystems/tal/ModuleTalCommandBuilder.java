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
		return module.asSubsystemCommand(new InstantCommand(() -> module.setDrivePower(power.get()), module), "set the motors power");
	}

	public Command steerToAngle(Supplier<Double> angle) {
		return module.asSubsystemCommand(new InstantCommand(() -> module.steerToAngle(angle.get()), module), "steer to an angle");
	}

	public Command changeNeutralMode() {
		return module.asSubsystemCommand(new InstantCommand(module::changeNeutralModeMotors, module), "change module neutral mode");
	}

	public Command FullyCircleDrive() {
		return module.asSubsystemCommand(
			new SequentialCommandGroup(
				new DriveDistanceCommand(module, 0, 2),
				new DriveDistanceCommand(module, 90, 2),
				new DriveDistanceCommand(module, 180, 2),
				new DriveDistanceCommand(module, 270, 2)
			),
			"drive 2 rotations to each 90 degrees angle"
		);
	}


}
