package frc.robot.subsystems.lifter;

import edu.wpi.first.wpilibj2.command.Command;

public class LifterStateHandler {

	private final Lifter lifter;

	public LifterStateHandler(Lifter lifter) {
		this.lifter = lifter;
	}

	public Command setState(LifterState state) {
		return switch (state) {
			case HOLD -> lifter.getCommandsBuilder().stop();
			case FORWARD -> lifter.getCommandsBuilder().setPower(LifterConstants.FORWARD_POWER);
			case BACKWARD -> lifter.getCommandsBuilder().setPower(LifterConstants.BACKWARD_POWER);
			case EXTENDED ->
				lifter.getCommandsBuilder()
					.setPower(LifterConstants.EXTENDING_POWER)
					.until(() -> lifter.isHigher(LifterConstants.EXTENDED_POSITION));
			case RETRACTED ->
				lifter.getCommandsBuilder()
					.setPower(LifterConstants.RETRACTING_POWER)
					.until(() -> lifter.isLower(LifterConstants.RETRACTED_POSITION));
		};
	}

}
