package frc.robot.subsystems.algaeIntake.rollers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.GBCommandsBuilder;

public class RollersCommandsBuilder extends GBCommandsBuilder {

	private final Rollers rollers;

	public RollersCommandsBuilder(Rollers rollers) {
		super();
		this.rollers = rollers;
	}

	public Command setPower(double power) {
		return rollers.asSubsystemCommand(new RunCommand(() -> rollers.setPower(power)), "Set power to " + power);
	}

	public Command stop() {
		return rollers.asSubsystemCommand(new RunCommand(rollers::stop), "Stop");
	}

}
