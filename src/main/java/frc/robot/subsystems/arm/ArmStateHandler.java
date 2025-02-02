package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class ArmStateHandler {

	private final Arm arm;
	private ArmState currentState;

	public ArmStateHandler(Arm arm) {
		this.arm = arm;
	}

	public ArmState getCurrentState() {
		return currentState;
	}

	public Command setState(ArmState state) {
		return new ParallelCommandGroup(
			new InstantCommand(() -> currentState = state),
			arm.getCommandsBuilder().moveToPosition(state.getPosition())
		);
	}

}
