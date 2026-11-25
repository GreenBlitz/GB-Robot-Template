package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.utils.utilcommands.InitExecuteCommand;

import java.util.function.Supplier;

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

	public Command goToPositionWithVelocity(Supplier<ElevatorMeasure> position,Supplier<ElevatorMeasure> velocityChangePerSecond){
		return elevator.asSubsystemCommand(
				new RunCommand(()->{
					elevator.updateTargetPosition(position.get());
					elevator.goToPosition(position.get(), velocityChangePerSecond.get());
				}
				),"Go to position from supplier"
		);
	}

	public Command setVoltage(double voltage){
		return elevator.asSubsystemCommand(
				new RunCommand(
						()->elevator.setVoltage(voltage)
				), "Set voltage: "+ voltage
		);
	}

	public Command setVoltage(Supplier<Double> voltage){
		return elevator.asSubsystemCommand(
				new RunCommand(
						()->{elevator.setVoltage(voltage.get());}
				), "Set voltage from supplier"
		);
	}

    public Command stop(){
        return elevator.asSubsystemCommand(
                new InstantCommand(elevator::stop), "stop"
        );
    }

}
