package frc.robot.subsystems.elevator;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.utils.utilcommands.InitExecuteCommand;

public class ElevatorCommandsBuilder {
    private final Elevator elevator;
    protected ElevatorCommandsBuilder(Elevator elevator){
        this.elevator=elevator;
    }
    public Command goToPositionWithVelocity(double positionFrac, double fracPositionPerSecond, double fractionalTolerance){
        return elevator.asSubsystemCommand(
                new InitExecuteCommand(
                        ()->{elevator.updateTargetPosition(positionFrac);},
                        ()->elevator.goToPosition(positionFrac, fracPositionPerSecond)
                ).until(()->{return elevator.isAtPosition(positionFrac,fractionalTolerance);}),"Go to position: "+ positionFrac
        );
    }
    public Command goToPositionWithVelocity(Distance position, Distance heightPerSecond, Distance tolerance){
        return elevator.asSubsystemCommand(
                new InitExecuteCommand(
                        ()->{elevator.updateTargetPosition(position);},
                        ()->elevator.goToPosition(position, heightPerSecond)
                ).until(()->{return elevator.isAtPosition(tolerance,tolerance);}),"Go to position: "+ position
        );
    }
}
