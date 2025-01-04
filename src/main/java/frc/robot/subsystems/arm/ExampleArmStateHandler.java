package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;

public class ExampleArmStateHandler {

	private final ExampleArm arm;

	public ExampleArmStateHandler(ExampleArm arm) {
		this.arm = arm;
	}

	public Command setState(ExampleArmState state) {
		if (state.equals(ExampleArmState.STAY_IN_PLACE)) {
			return arm.getCommandBuilder().stayInPlace();
		}
		return arm.getCommandBuilder().moveToPosition(state.getPosition());
	}

	public Command endSate(ExampleArmState state) {
		return switch (state) {
			case HIGH_DROP, MID_DROP -> setState(ExampleArmState.LOW_DROP);
			case SAFE_HOLD -> setState(ExampleArmState.STAY_IN_PLACE);
			case INTAKE, STAY_IN_PLACE, LOW_DROP -> setState(ExampleArmState.SAFE_HOLD);
		};
	}


}
