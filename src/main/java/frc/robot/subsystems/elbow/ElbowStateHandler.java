package frc.robot.subsystems.elbow;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ElbowStateHandler {

	private final Elbow elbow;

	public ElbowStateHandler(Elbow elbow) {
		this.elbow = elbow;
	}

	public Command setState(ElbowState elbowState) {
		if (elbowState == ElbowState.MANUAL) {
			return new InstantCommand();
		}
		if(elbowState == ElbowState.FREE){
			return elbow.getCommandsBuilder().setPower(() -> 0).alongWith(new InstantCommand(() -> elbow.setBrake(false)));
		}
		return elbow.getCommandsBuilder().moveToAngle(elbowState.getTargetPosition()).alongWith(new InstantCommand(() -> elbow.setBrake(false)));
	}

}
