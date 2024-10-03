package frc.robot.subsystems.elbow;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ElbowStateHandler {

	private static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(2);

	private final Elbow elbow;

	public ElbowStateHandler(Elbow elbow) {
		this.elbow = elbow;
	}

	public Command setState(ElbowState elbowState) {
		if (elbowState == ElbowState.MANUAL) {
			return new InstantCommand();
		}
		if(elbowState == ElbowState.FREE){
			return new InstantCommand(() -> elbow.setPower(0), elbow);
		}
		if(elbowState == ElbowState.CLIMB){
			return elbow.getCommandsBuilder().moveToAngle(elbowState.getTargetPosition(), TOLERANCE)
					.alongWith(new InstantCommand(() -> elbow.setBrake(false)));
		}
		return elbow.getCommandsBuilder().moveToAngle(elbowState.getTargetPosition(), TOLERANCE).alongWith(new InstantCommand(() -> elbow.setBrake(true)));
	}

}
