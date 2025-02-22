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
			case FORWARD, BACKWARD -> lifter.getCommandsBuilder().setPower(state.getPower());
			case DEPLOY -> lifter.getCommandsBuilder().setPower(state.getPower()).until(() -> lifter.isHigher(state.getTargetPosition()));
			case CLIMB -> lifter.getCommandsBuilder().setPower(state.getPower()).until(() -> lifter.isLower(state.getTargetPosition()));
		};
	}

	public boolean isHigher(Rotation2d position) {
		return lifter.isHigher(position);
	}

	public boolean isLower(Rotation2d position) {
		return lifter.isLower(position);
	}

}
