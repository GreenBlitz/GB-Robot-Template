package frc.robot.subsystems.funnel;

import edu.wpi.first.wpilibj2.command.Command;

public class FunnelStateHandler {

	private final Funnel funnel;

	public FunnelStateHandler(Funnel funnel) {
		this.funnel = funnel;
	}

	public Command setState(FunnelState funnelState) {
		return funnel.getCommandBuilder().setPower(funnelState.getPower());
	}

}
