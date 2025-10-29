package frc.robot.subsystems.algaeIntake.rollers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class RollersCommandsBuilder {

	private final Rollers rollers;
	private boolean isRunningIndependently;

	public RollersCommandsBuilder(Rollers rollers) {
		this.rollers = rollers;
		this.isRunningIndependently = false;
	}

	public boolean isRunningIndependently() {
		return isRunningIndependently;
	}

	public void setIsRunningIndependently(boolean isRunningIndependently) {
		this.isRunningIndependently = isRunningIndependently;
	}

	public Command setPower(double power) {
		return rollers.asSubsystemCommand(new RunCommand(() -> rollers.setPower(power)), "Set power to " + power);
	}

	public Command stop() {
		return rollers.asSubsystemCommand(new RunCommand(rollers::stop), "Stop");
	}

}
