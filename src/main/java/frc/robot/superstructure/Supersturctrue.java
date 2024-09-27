package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.subsystems.elbow.ElbowState;
import frc.robot.subsystems.elbow.ElbowStateHandler;
import frc.robot.subsystems.flywheel.FlywheelState;
import frc.robot.subsystems.flywheel.FlywheelStateHandler;
import frc.robot.subsystems.funnel.FunnelState;
import frc.robot.subsystems.funnel.FunnelStateHandler;
import frc.robot.subsystems.intake.IntakeState;
import frc.robot.subsystems.intake.IntakeStateHandler;
import frc.robot.subsystems.pivot.PivotState;
import frc.robot.subsystems.pivot.PivotStateHandler;
import frc.robot.subsystems.swerve.Swerve;

import javax.print.DocFlavor;

public class Supersturctrue {

    private final Robot robot;

    private final Swerve swerve;
    private final ElbowStateHandler elbowStateHandler;
    private final FlywheelStateHandler flywheelStateHandler;
    private final FunnelStateHandler funnelStateHandler;
    private final IntakeStateHandler intakeStateHandler;
    private final PivotStateHandler pivotStateHandler;

    private RobotState currentState;

    public Supersturctrue(Robot robot) {
        this.robot = robot;
        this.swerve = robot.getSwerve();
        this.elbowStateHandler = new ElbowStateHandler(robot.getElbow());
        this.flywheelStateHandler = new FlywheelStateHandler(robot.getFlywheel());
        this.funnelStateHandler = new FunnelStateHandler(robot.getFunnel());
        this.intakeStateHandler = new IntakeStateHandler(robot.getIntake());
        this.pivotStateHandler = new PivotStateHandler(robot.getPivot());
    }

    public Command setState(RobotState state) {
        this.currentState = state;
        return switch (state) {
            case IDLE -> idle();
            case PRE_SPEAKER -> preSpeaker();
            case PRE_AMP -> preAMP();
            case TRANSFER_SHOOTER_ARM -> null;
            case TRANSFER_ARM_SHOOTER -> null;
            case INTAKE -> null;
            case SPEAKER -> null;
            case AMP -> null;
        };
    }

    private Command idle() {
        return new ParallelCommandGroup(
                pivotStateHandler.setState(PivotState.IDLE),
                flywheelStateHandler.setState(FlywheelState.DEFAULT),
                elbowStateHandler.setState(ElbowState.IDLE),
                intakeStateHandler.setState(IntakeState.STOP),
                funnelStateHandler.setState(FunnelState.STOP)
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

    private Command transferShooterArm() {
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        pivotStateHandler.setState(PivotState.TRANSFER),
                        elbowStateHandler.setState(ElbowState.TRANSFER)
                ),
                funnelStateHandler.setState(FunnelState.TRANSFER_TO_ARM)
                //add roller
        );
    }

}
