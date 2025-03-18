package frc.robot.subsystems.climb.lifter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class LifterStateHandler {

	private final Lifter lifter;

	public LifterStateHandler(Lifter lifter) {
		this.lifter = lifter;
	}

	public Command setState(LifterState state) {
		return switch (state) {
			case HOLD -> lifter.getCommandsBuilder().stop();
			case FORWARD, BACKWARD, DEPLOY, CLIMB_WITHOUT_LIMIT_SWITCH, MANUAL_CLIMB, CLOSE ->
				lifter.getCommandsBuilder().setPower(state.getPower());
			case CLIMB_WITH_LIMIT_SWITCH -> lifter.getCommandsBuilder().climbWithLimitSwitch();
		};
	}

	public boolean isHigher(Rotation2d position) {
		return lifter.isHigher(position);
	}

	public boolean isLower(Rotation2d position) {
		return lifter.isLower(position);
	}

}
