package frc.robot.statemachine.superstructure;

import frc.robot.Robot;
import frc.robot.statemachine.Tolerances;
import frc.robot.subsystems.arm.ArmStateHandler;
import frc.robot.subsystems.elevator.ElevatorStateHandler;
import frc.robot.subsystems.endeffector.EndEffectorStateHandler;

public class Superstructure {

    private final String logPath;
    private final Robot robot;
    private final ElevatorStateHandler elevatorStateHandler;
    private final ArmStateHandler armStateHandler;
    private final EndEffectorStateHandler endEffectorStateHandler;

    public Superstructure(String logPath, Robot robot) {
        this.logPath = logPath;
        this.robot = robot;
        this.elevatorStateHandler = new ElevatorStateHandler(robot.getElevator());
        this.armStateHandler = new ArmStateHandler(robot.getArm());
        this.endEffectorStateHandler = new EndEffectorStateHandler(robot.getEndEffector());
    }

    public boolean isCoralIn() {
        return robot.getEndEffector().isCoralInBack();
    }

    public boolean isCoralOut() {
        return !robot.getEndEffector().isCoralInFront();
    }

    public boolean isReadyToScore(ScoreLevel scoreLevel) {
        return robot.getElevator().isAtPosition(scoreLevel.getElevatorPositionMeters(), Tolerances.ELEVATOR_HEIGHT_METERS)
            && robot.getArm().isAtPosition(scoreLevel.getArmPosition(), Tolerances.ARM_POSITION);
    }

}
