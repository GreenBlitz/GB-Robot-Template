package frc.robot.subsystems.arm;

import edu.wpi.first.wpilibj2.command.Command;

public class ArmStateHandler {

	private final Arm arm;

	public ArmStateHandler(Arm arm) {
		this.arm = arm;
	}

	public Command setState(ArmState state) {
		if (state.equals(ArmState.STAY_IN_PLACE)) {
			return arm.getCommandBuilder().stayInPlace();
		}
		return arm.getCommandBuilder().moveToPosition(state.getPosition());
	}

	public Command endSate(ArmState state){
		switch (state){
            case INTAKE -> {
                return setState(ArmState.SAFE_HOLD);
            }
			case SAFE_HOLD -> {
				return setState(ArmState.STAY_IN_PLACE);
			}
			//need to be completed


        }
	}


}
