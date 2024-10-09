package frc.robot.subsystems.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class FlywheelStateHandler {

	private final Flywheel flywheel;

	public FlywheelStateHandler(Robot robot) {
		this.flywheel = robot.getFlywheel();
	}

	public Command setState(FlywheelState state) {
		if (state.getVelocity().getRotations() == 0) {
			return flywheel.getCommandsBuilder().stop();
		} else {
			return flywheel.getCommandsBuilder().setVelocity(state.getVelocity());
		}
	}

}
