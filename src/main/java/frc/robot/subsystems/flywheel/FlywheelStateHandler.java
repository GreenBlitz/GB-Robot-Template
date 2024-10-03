package frc.robot.subsystems.flywheel;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class FlywheelStateHandler {

	private final Flywheel flywheel;

	public FlywheelStateHandler(Robot robot) {
		this.flywheel = robot.getFlywheel();
	}

	public Command setState(FlywheelState state) {
		return flywheel.getCommandsBuilder().setVelocity(state.getPower());
	}

}
