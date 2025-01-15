package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorStateHandler {

    private final Elevator elevator;

    public ElevatorStateHandler(Elevator elevator){
        this.elevator = elevator;
    }

    public Command setState(ElevatorState state){
        if (state == ElevatorState.IDLE) {
            return elevator.getCommandsBuilder().stop();
        }
        return elevator.getCommandsBuilder().setTargetPositionMeters(state.getMeters());
    }

}