package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class Superstructure {

    private final Robot robot;
    private RobotState currentState;

    public Superstructure(Robot robot) {
        this.robot = robot;
    }

    public RobotState getCurrentState() {
        return currentState;
    }

    public void logStatus() {
        Logger.recordOutput("CurrentState", currentState);
    }

    public Command setState(RobotState state) {
        return switch (state) {
            case IDLE -> null;
            case INTAKE -> null;
            case PRE_SPEAKER -> null;
            case SPEAKER -> null;
            case PRE_AMP -> null;
            case AMP -> null;
            case TRANSFER_SHOOTER_ELEVATOR -> null;
            case TRANSFER_ELEVATOR_SHOOTER -> null;
            case SHOOTER_OUTTAKE -> null;
        };
    }

}
