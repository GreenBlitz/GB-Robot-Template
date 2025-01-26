package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;

public class ArmStateHandler {

	private final Arm arm;

	public ArmStateHandler(Arm arm) {
		this.arm = arm;
	}

	public Command setState(ArmState state) {
		return arm.getCommandsBuilder().moveToPosition(state.getPosition());
	}

}
