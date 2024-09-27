package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.flywheel.FlywheelState;
import frc.robot.subsystems.flywheel.FlywheelStateHandler;
import frc.robot.subsystems.pivot.PivotState;
import frc.robot.subsystems.pivot.PivotStateHandler;

public class Supersturctrue {

    private final Robot robot;

    private final PivotStateHandler pivotStateHandler;
    private final FlywheelStateHandler flywheelStateHandler;

    public Supersturctrue(Robot robot) {
        this.robot = robot;
        this.pivotStateHandler = new PivotStateHandler(robot.getPivot());
        this.flywheelStateHandler = new FlywheelStateHandler(robot.getFlywheel());
    }

    public Command setState(RobotState state) {
        return switch (state) {
            case PRE_SPEAKER -> preSpeaker();
            case IDLE ->
        }
    }

    private Command idle() {
        return new ParallelCommandGroup(

        )
    }

    private Command preSpeaker() {
        return new ParallelCommandGroup(
            pivotStateHandler.setState(PivotState.SHOOT),
            flywheelStateHandler.setState(FlywheelState.SHOOT)
        );
    }

}
