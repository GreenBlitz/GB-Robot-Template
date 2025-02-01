package frc.robot.statemachine;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.constants.field.enums.Branch;
import frc.robot.Robot;
import frc.robot.scoringhelpers.ScoringHelpers;
import frc.robot.statemachine.superstructure.ScoreLevel;
import frc.robot.statemachine.superstructure.Superstructure;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveMath;
import frc.robot.subsystems.swerve.states.SwerveState;
import frc.robot.subsystems.swerve.states.aimassist.AimAssist;
import frc.utils.math.ToleranceMath;

public class DaddyRobot {

    private final Robot robot;
    private final Swerve swerve;
    private final Superstructure superstructure;

    public DaddyRobot(Robot robot){
        this.robot = robot;
        this.swerve = robot.getSwerve();
        this.superstructure = robot.getSuperstructure();
    }

    private boolean isReadyToScore(ScoreLevel level, Branch branch){
        return superstructure.isReadyToScore(level) && isAtPose(robot.getPoseEstimator().getEstimatedPose(), ScoringHelpers.getRobotScoringPose(branch, StateMachineConstants.ROBOT_SCORING_DISTANCE_FROM_REEF_METERS), swerve.getRobotRelativeVelocity(), );
    }

    public boolean isAtPose(Pose2d currentPose, Pose2d targetPose, ChassisSpeeds currentSpeeds, Pose2d tolerances, Pose2d deadbands) {
        boolean isAtX = MathUtil.isNear(targetPose.getX(), currentPose.getX(), tolerances.getX());
        boolean isAtY = MathUtil.isNear(targetPose.getY(), currentPose.getY(), tolerances.getY());
        boolean isAtHeading = ToleranceMath.isNearWrapped(targetPose.getRotation(), currentPose.getRotation(), tolerances.getRotation());
        boolean isStopping = SwerveMath.isStill(currentSpeeds, deadbands);
        return isAtX && isAtY && isAtHeading && isStopping;
    }

    public Command setState(RobotState state){
        return switch (state){
            case DRIVE -> drive();
            case INTAKE -> intake();
            case L1 -> l1();
            case L2 -> l2();
            case L3 -> l3();
            case L4 -> l4();
            case PRE_L1 -> preL1();
            case PRE_L2 -> preL2();
            case PRE_L3 -> preL3();
            case PRE_L4 -> preL4();
            case OUTTAKE -> outtake();
            case ALIGN_REEF -> alignReef();
        };
    }

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
            superstructure.scoreL1(),
            swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.REEF))
        );
    }

    private Command l2(){
        return new ParallelCommandGroup(
            superstructure.scoreL2(),
            swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.BRANCH))
        );
    }

    private Command l3(){
        return new ParallelCommandGroup(
            superstructure.scoreL3(),
            swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.BRANCH))
        );
    }

    private Command l4(){
        return new SequentialCommandGroup(
                new ParallelCommandGroup(
                        swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.BRANCH)),
                        superstructure.preL4()
                ).until()
                superstructure.scoreL4()
        );
    }

    private Command preL1(){
        return new ParallelCommandGroup(
            superstructure.preL1(),
            swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.REEF))
        );
    }

    private Command preL2(){
        return new ParallelCommandGroup(
            superstructure.preL2(),
            swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.BRANCH))
        );
    }

    private Command preL3(){
        return new ParallelCommandGroup(
            superstructure.preL3(),
            swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.BRANCH))
        );
    }

    private Command preL4(){
        return new ParallelCommandGroup(
            superstructure.preL4(),
            swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.BRANCH))
        );
    }

    private Command outtake(){
        return new ParallelCommandGroup(
            superstructure.outtake(),
            swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE)
        );
    }

    private Command alignReef(){
        return new ParallelCommandGroup(
            superstructure.idle(),
            swerve.getCommandsBuilder().driveByDriversInputs(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.REEF))
        );
    }

}
