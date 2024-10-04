package frc.robot.subsystems.funnel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class FunnelStateHandler {

	private final Funnel funnel;

	public FunnelStateHandler(Robot robot) {
		this.funnel = robot.getFunnel();
	}

	public Command setState(FunnelState state) {
		if (state == FunnelState.STOP) {
			return funnel.getCommandsBuilder().stop();
		}
		return funnel.getCommandsBuilder().setPower(state.getPower());
	}

}
