package frc.robot.subsystems.endeffector;

import edu.wpi.first.wpilibj2.command.Command;

public class EndEffectorStateHandler {

	private final EndEffector endEffector;

	public EndEffectorStateHandler(EndEffector endEffector) {
		this.endEffector = endEffector;
	}

	public Command setState(EndEffectorState state) {
		return endEffector.getCommandsBuilder().setPower(state.getPower());
	}

}
