package frc.robot.subsystems.funnel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class FunnelCommands {

	private Funnel funnel;

	public FunnelCommands(Funnel funnel) {
		this.funnel = funnel;
	}

	public Command setPower(double power) {
		return new FunctionalCommand(() -> {}, () -> funnel.setPower(power), (interrupted) -> funnel.stop(), () -> false, funnel);
	}

	public Command stop() {
		return new InstantCommand(() -> funnel.stop(), funnel);
	}


}
