package frc.robot.subsystems.tal;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;

import java.util.function.Supplier;

public class GBModuleTalCommandBuilder {

	private final GBModuleTal module;

	public GBModuleTalCommandBuilder(GBModuleTal module) {
		this.module = module;
		this.module.setDefaultCommand(stop());
	}

	public Command driveModuleByJoystick(Supplier<Rotation2d> angle, Supplier<Double> power) {
		return module
			.asSubsystemCommand(new RunCommand(() -> module.drive(angle.get(), power.get()), module), "drive module by power and angle");
	}

	public Command stop() {
		return module.asSubsystemCommand(new RunCommand(module::stop), "stop motors");
	}

	public Command setDrivePower(Supplier<Double> power) {
		return module.asSubsystemCommand(new InstantCommand(() -> module.setDrivePower(power.get()), module), "set the motors power");
	}

	public Command steerToAngle(Supplier<Rotation2d> angle) {
		return module.asSubsystemCommand(new InstantCommand(() -> module.steerToUnrestrictedAngle(angle.get()), module), "steer to an angle");
	}

	public Command changeNeutralMode() {
		return module.asSubsystemCommand(new InstantCommand(module::changeMotorsBrake, module), "change module neutral mode");
	}

	public Command FullyCircleDrive() {
		return module.asSubsystemCommand(
			new SequentialCommandGroup(
				new DriveDistanceCommand(module, Rotation2d.fromDegrees(this.module.getSteerAngle().getDegrees()), new Rotation2d(2 * Math.PI)),
				new DriveDistanceCommand(
					module,
					new Rotation2d(this.module.getSteerAngle().getRadians() + (1.0 / 2) * Math.PI),
					new Rotation2d(2 * Math.PI)
				),
				new DriveDistanceCommand(
					module,
					new Rotation2d(this.module.getSteerAngle().getRadians() + Math.PI),
					new Rotation2d(2 * Math.PI)
				),
				new DriveDistanceCommand(
					module,
					new Rotation2d(this.module.getSteerAngle().getRadians() + Math.PI + (1.0 / 2) * Math.PI),
					new Rotation2d(2 * Math.PI)
				)
			),
			"drive 2 rotations to each 90 degrees angle"
		);
	}

}
