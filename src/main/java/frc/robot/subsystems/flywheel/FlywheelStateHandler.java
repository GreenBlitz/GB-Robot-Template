package frc.robot.subsystems.flywheel;

import edu.wpi.first.wpilibj2.command.Command;

public class FlywheelStateHandler {

	private final Flywheel flywheel;

	public FlywheelStateHandler(Flywheel flywheel) {
		this.flywheel = flywheel;
	}

	public Command setState(FlywheelState flywheelState) {
		if (flywheelState.getRightVelocity().getRotations() == 0) {
			return flywheel.getCommandsBuilder().stop();
		}
		return flywheel.getCommandsBuilder().setVelocities(flywheelState.getRightVelocity(), flywheelState.getLeftVelocity());
	}

}
