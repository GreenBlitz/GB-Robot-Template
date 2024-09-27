package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.constants.Field;
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
import frc.robot.subsystems.swerve.SwerveState;
import frc.robot.subsystems.swerve.swervestatehelpers.AimAssist;

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

    public RobotState getCurrentState() {
        return currentState;
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
            case OUTTAKE -> null;
        };
    }

    private Command idle() {
        return new ParallelCommandGroup(
            pivotStateHandler.setState(PivotState.IDLE),
            flywheelStateHandler.setState(FlywheelState.DEFAULT),
            elbowStateHandler.setState(ElbowState.IDLE),
            intakeStateHandler.setState(IntakeState.STOP),
            funnelStateHandler.setState(FunnelState.STOP),
            swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE)
        );
    }

    private Command preSpeaker() {
        return new ParallelCommandGroup(
            funnelStateHandler.setState(FunnelState.STOP),
            intakeStateHandler.setState(IntakeState.STOP),
            pivotStateHandler.setState(PivotState.PRE_SPEAKER),
            flywheelStateHandler.setState(FlywheelState.PRE_SPEAKER),
            elbowStateHandler.setState(ElbowState.IDLE),
            swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.SPEAKER))
        );
    }

    private Command preAMP() {
        return new ParallelCommandGroup(
                pivotStateHandler.setState(PivotState.IDLE),
                flywheelStateHandler.setState(FlywheelState.DEFAULT),
                new SequentialCommandGroup(
                        new ParallelDeadlineGroup(
                            swerve.getCommandsBuilder().saveState(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.AMP)),
                            new RunCommand(() -> {}).until(() -> swerve.isAtHeading(Field.getAngleToAmp()))
                        ),
                        new ParallelCommandGroup(
                            funnelStateHandler.setState(FunnelState.RELEASE_FOR_ARM),
                            elbowStateHandler.setState(ElbowState.PRE_AMP)
                        )
                )
        );
    }

    private Command transferShooterArm() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                    pivotStateHandler.
            )
        );
    }

}
