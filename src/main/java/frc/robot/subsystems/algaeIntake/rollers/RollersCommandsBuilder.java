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
		return new RunCommand(() -> rollers.setPower(power));
	}

	public Command setVelocity(Rotation2d targetVelocityMPS) {
		return new RunCommand(() -> rollers.setTargetVelocity(targetVelocityMPS));
	}

	public Command setVoltage(double voltage) {
		return new RunCommand(() -> rollers.setVoltage(voltage));
	}

}
