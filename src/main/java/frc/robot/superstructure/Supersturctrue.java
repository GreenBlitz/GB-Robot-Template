package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.pivot.PivotState;
import frc.robot.subsystems.pivot.PivotStateHandler;

public class Supersturctrue {

    private final Robot robot;

    private final PivotStateHandler pivotStateHandler;

    public Supersturctrue(Robot robot) {
        this.robot = robot;
        this.pivotStateHandler = new PivotStateHandler(robot.getPivot());
    }

    public void setState(RobotState state) {

    }

    private Command preSpeaker() {
        return new ParallelCommandGroup(
                pivotStateHandler.setState(PivotState.SHOOT),

        );
    }

}
