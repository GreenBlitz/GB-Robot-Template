package frc.robot.statemachine;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Robot;
import frc.robot.statemachine.superstructure.Superstructure;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.states.SwerveState;
import frc.robot.subsystems.swerve.states.aimassist.AimAssist;

public class DaddyRobot {

    private final Swerve swerve;
    private final Superstructure superstructure;

    public DaddyRobot(Robot robot){
        this.swerve = robot.getSwerve();
        this.superstructure = robot.getSuperstructure();
    }

    public Command setState(RobotState state){
        return switch (state){
            case DRIVE -> drive();
            case INTAKE -> intake();
            case L1 -> l1();
            case L2 -> l3();
            case L3 -> ;
            case L4 -> ;
            case PRE_L1 -> ;
            case PRE_L2 -> ;
            case PRE_L3 -> ;
            case PRE_L4 -> ;
            case OUTTAKE -> ;
            case ALIGN_REEF -> ;
        };
    }

    //@formatter:off
    private Command drive(){
        return new ParallelCommandGroup(
                superstructure.idle(),
                swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)
        );
    }

    private Command intake(){
        return new ParallelCommandGroup(
                superstructure.intake(),
                swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.CORAL_STATION))
        );
    }

    private Command l1(){
        return new ParallelCommandGroup(
                superstructure.l1(),
                swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.REEF))
        );
    }

    private Command l2(){
        return new ParallelCommandGroup(
                superstructure.l2(),
                swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.BRANCH))
        );
    }

    private Command l3(){
        return new ParallelCommandGroup(
                superstructure.l3(),
                swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.BRANCH))
        );
    }

}
