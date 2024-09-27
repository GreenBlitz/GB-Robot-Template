package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.elbow.ElbowState;
import frc.robot.subsystems.elbow.ElbowStateHandler;
import frc.robot.subsystems.flywheel.FlywheelState;
import frc.robot.subsystems.flywheel.FlywheelStateHandler;
import frc.robot.subsystems.pivot.PivotState;
import frc.robot.subsystems.pivot.PivotStateHandler;
import frc.robot.subsystems.swerve.Swerve;

public class Supersturctrue {

    private final Robot robot;

    private final Swerve swerve;
    private final PivotStateHandler pivotStateHandler;
    private final FlywheelStateHandler flywheelStateHandler;
    private final ElbowStateHandler elbowStateHandler;

    private RobotState currentState;

    public Supersturctrue(Robot robot) {
        this.robot = robot;
        this.swerve = robot.getSwerve();
        this.pivotStateHandler = new PivotStateHandler(robot.getPivot());
        this.flywheelStateHandler = new FlywheelStateHandler(robot.getFlywheel());
        this.elbowStateHandler = new ElbowStateHandler(robot.getElbow());
    }

    public Command setState(RobotState state) {
        this.currentState = state;
        return switch (state) {
            case IDLE -> idle();
            case PRE_SPEAKER -> preSpeaker();
            case PRE_AMP -> preAMP();
        };
    }

    private Command idle() {
        return new ParallelCommandGroup(
            pivotStateHandler.setState(PivotState.IDLE),
            flywheelStateHandler.setState(FlywheelState.DEFAULT),
            elbowStateHandler.setState(ElbowState.IDLE)
        );
    }

    private Command preSpeaker() {
        return new ParallelCommandGroup(
            pivotStateHandler.setState(PivotState.PRE_SPEAKER),
            flywheelStateHandler.setState(FlywheelState.PRE_SPEAKER),
            elbowStateHandler.setState(ElbowState.IDLE)
        );
    }

    private Command preAMP() {
        return new ParallelCommandGroup(
            pivotStateHandler.setState(PivotState.IDLE),
            flywheelStateHandler.setState(FlywheelState.DEFAULT),
            new SequentialCommandGroup(
                    // add swerve turn
                    elbowStateHandler.setState(ElbowState.PRE_AMP)
            )
        );
    }

}
