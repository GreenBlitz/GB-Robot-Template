package frc.robot.subsystems.algaeIntake.rollers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class RollersCommandsBuilder {

	private final Rollers rollers;

	public RollersCommandsBuilder(Rollers rollers) {
		this.rollers = rollers;
	}

	public Command setPower(double power) {
		return rollers.asSubsystemCommand(new RunCommand(() -> rollers.setPower(power)), "Set power to " + power);
	}

	public Command stop() {
		return rollers.asSubsystemCommand(new RunCommand(rollers::stop), "Stop");
	}


	public Command setVelocity(Rotation2d targetVelocityMPS) {
		return rollers
			.asSubsystemCommand(new RunCommand(() -> rollers.setTargetVelocity(targetVelocityMPS)), "Set velocity to " + targetVelocityMPS);
	}

	public Command setVoltage(double voltage) {
		return rollers.asSubsystemCommand(new RunCommand(() -> rollers.setVoltage(voltage)), "Set voltage to " + voltage);
	}

}
