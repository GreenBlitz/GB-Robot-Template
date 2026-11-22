package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.utils.utilcommands.InitExecuteCommand;

public class ElevatorCommandsBuilder {

	private final Elevator elevator;

	protected ElevatorCommandsBuilder(Elevator elevator) {
		this.elevator = elevator;
	}

	public Command goToPositionWithVelocity(ElevatorMeasure position, ElevatorMeasure changePerSecond, ElevatorMeasure tolerance) {
		return elevator.asSubsystemCommand(new InitExecuteCommand(() -> {
			elevator.updateTargetPosition(position);
		}, () -> elevator.goToPosition(position, changePerSecond)).until(() -> {
			return elevator.isAtPosition(position, tolerance);
		}), "Go to position: " + position);
	}

    public Command stop(){
        return elevator.asSubsystemCommand(
                new InstantCommand(elevator::stop), "stop"
        );
    }

}
