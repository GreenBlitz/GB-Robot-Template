package frc.robot.subsystems.flywheel;

import edu.wpi.first.wpilibj2.command.Command;

public class FlywheelStateHandler {

	private final Flywheel flywheel;

	public FlywheelStateHandler(Flywheel flywheel) {
		this.flywheel = flywheel;
	}

	public Command setState(FlywheelState state) {
		if (state.getVelocity().getRotations() == 0) {
			return flywheel.getCommandsBuilder().stop();
		} else {
			return flywheel.getCommandsBuilder().setVelocity(state.getVelocity());
		}
	}

}
