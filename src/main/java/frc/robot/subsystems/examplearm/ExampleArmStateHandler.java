package frc.robot.subsystems.examplearm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

public class ExampleArmStateHandler {

	private final ExampleArm arm;

	public ExampleArmStateHandler(ExampleArm arm) {
		this.arm = arm;
	}

	public Command setState(ExampleArmState state) {
		return switch (state){
			case STAY_IN_PLACE -> arm.getCommandBuilder().stayInPlace();
			case INTAKE, SAFE_HOLD, LOW_DROP,MID_DROP, HIGH_DROP -> arm.getCommandBuilder().moveToPosition(state.getPosition());
		};
	}

	public Command endSate(ExampleArmState state) {
		return switch (state) {
			case SAFE_HOLD, HIGH_DROP, MID_DROP, LOW_DROP, INTAKE, STAY_IN_PLACE  -> setState(ExampleArmState.STAY_IN_PLACE);
		};
	}


}
