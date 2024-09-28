package frc.robot.subsystems.funnel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class FunnelStateHandler {

	private final Funnel funnel;

	public FunnelStateHandler(Funnel funnel) {
		this.funnel = funnel;
	}

	public Command setState(FunnelState funnelState) {
		if (funnelState == FunnelState.MANUAL) {
			return new InstantCommand();
		}
		return funnel.getCommandsBuilder().setPower(funnelState.getPower());
	}

}
