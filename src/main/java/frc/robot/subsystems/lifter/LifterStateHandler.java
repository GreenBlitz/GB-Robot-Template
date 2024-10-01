package frc.robot.subsystems.lifter;

import edu.wpi.first.wpilibj2.command.Command;

public class LifterStateHandler {

	private final Lifter lifter;

	public LifterStateHandler(Lifter lifter) {
		this.lifter = lifter;
	}

	public Command setState(LifterState state) {
		return switch (state) {
			case STOP -> lifter.getLifterCommandsBuilder().stop();
			case FORWARD -> lifter.getLifterCommandsBuilder().setPower(LifterConstants.FORWARD_POWER);
			case BACKWARD -> lifter.getLifterCommandsBuilder().setPower(LifterConstants.BACKWARD_POWER);
			case EXTENDED -> lifter.getLifterCommandsBuilder().extend(LifterConstants.EXTENDED_POSITION);
			case RETRACTED -> lifter.getLifterCommandsBuilder().retract(LifterConstants.RETRACTED_POSITION);
		};
	}

}
