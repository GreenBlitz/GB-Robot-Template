package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.swervestatehelpers.AimAssist;
import frc.robot.subsystems.swerve.swervestatehelpers.DriveMode;
import frc.robot.subsystems.swerve.swervestatehelpers.DriveSpeed;
import frc.robot.subsystems.swerve.swervestatehelpers.RotateAxis;
import frc.utils.allianceutils.AlliancePose2d;
import frc.utils.allianceutils.AllianceRotation2d;
import frc.utils.pathplannerutils.PathPlannerUtils;
import frc.utils.utilcommands.InitExecuteCommand;

import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SwerveCommands {

    private static final Swerve SWERVE = RobotContainer.SWERVE;

    public static Command getLockSwerveCommand() {
        return new FunctionalCommand(
                () -> {},
                SWERVE::lockSwerve,
                inter -> {}, () -> false
        );
    }

    public static Command getPointWheelsCommand(Rotation2d wheelsAngle) {
        return new FunctionalCommand(
                () -> {},
                () -> SWERVE.pointWheels(wheelsAngle),
                inter -> {},
                () -> false
        );
    }

    public static Command getOpenLoopFieldRelativeDriveCommandSlow(
            DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier
    ) {
        return new InitExecuteCommand(
                () -> SWERVE.initializeDrive(new SwerveState().withDriveSpeed(DriveSpeed.SLOW)),
                () -> SWERVE.drive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSupplier.getAsDouble()),
                SWERVE
        );
    }

    public static Command getRotateToSpeaker(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier) {
        return new InitExecuteCommand(
                () -> SWERVE.initializeDrive(new SwerveState().withAimAssist(AimAssist.SPEAKER)),
                () -> SWERVE.drive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSupplier.getAsDouble()),
                SWERVE
        );
    }

    public static Command getRotateToAngleCommand(
            AllianceRotation2d targetAngle
    ) {
        return new InitExecuteCommand(
                () -> SWERVE.initializeDrive(new SwerveState()),
                () -> SWERVE.rotateToAngle(targetAngle),
                SWERVE
        );
    }

    public static Command getRotateToAngleCommand(
            AllianceRotation2d targetAngle, RotateAxis rotateAxis
    ) {
        return new InitExecuteCommand(
                () -> SWERVE.initializeDrive(new SwerveState().withRotateAxis(rotateAxis)),
                () -> SWERVE.rotateToAngle(targetAngle),
                SWERVE
        );
    }


    public static Command getDriveAroundWheelCommand(
            DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier, RotateAxis rotateAxis
    ) {
        return new InitExecuteCommand(
                () -> SWERVE.initializeDrive(new SwerveState().withRotateAxis(rotateAxis)),
                () -> SWERVE.drive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSupplier.getAsDouble()),
                SWERVE
        );
    }

    public static Command getSelfDriveCommand(
            DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier
    ) {
        return new InitExecuteCommand(
                () -> SWERVE.initializeDrive(new SwerveState().withDriveMode(DriveMode.SELF_RELATIVE)),
                () -> SWERVE.drive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSupplier.getAsDouble()),
                SWERVE
        );
    }

    /**
     * Creates a command that drives the swerve with the given powers, relative to the field's frame of reference, in closed
     * open mode.
     *
     * @param xSupplier the target forwards power
     * @param ySupplier the target leftwards power
     * @param thetaSupplier the target theta power, CCW+
     * @return the command
     */
    public static Command getOpenLoopFieldRelativeDriveCommand(
            DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier
    ) {
        return new InitExecuteCommand(
                () -> SWERVE.initializeDrive(new SwerveState()),
                () -> SWERVE.drive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSupplier.getAsDouble()),
                SWERVE
        );
    }


    public static Command getDriveToPoseCommand(Supplier<AlliancePose2d> targetPose, PathConstraints constraints) {
        return new DeferredCommand(() -> getCurrentDriveToPoseCommand(targetPose.get(), constraints), Set.of(SWERVE));
    }

    private static Command getCurrentDriveToPoseCommand(AlliancePose2d targetPose, PathConstraints constraints) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> SWERVE.initializeDrive(new SwerveState().withDriveMode(DriveMode.SELF_RELATIVE))),
                getPathfindToPoseCommand(targetPose, constraints),
                getPIDToPoseCommand(targetPose)
        );
    }

    private static Command getPathfindToPoseCommand(AlliancePose2d targetPose, PathConstraints pathConstraints) {
        Pose2d targetBluePose = targetPose.getBlueAlliancePose();
        Pose2d currentBluePose = RobotContainer.POSE_ESTIMATOR.getCurrentPose().getBlueAlliancePose();
        // todo - maybe move all func to "PathPlannerUtils"
        double distance = currentBluePose.getTranslation().getDistance(targetBluePose.getTranslation());
        //todo - understand why the if
        if (distance < SwerveConstants.CLOSE_TO_TARGET_POSITION_DEADBAND_METERS) {
            return PathPlannerUtils.createOnTheFlyPathCommand(
                    currentBluePose,
                    targetBluePose,
                    pathConstraints
            );
        }
        return AutoBuilder.pathfindToPose(targetBluePose, pathConstraints);
    }

    private static Command getPIDToPoseCommand(AlliancePose2d targetPose) {
        return new InstantCommand(SWERVE::resetRotationController).andThen(
                new RunCommand(() -> SWERVE.pidToPose(targetPose)).until(
                        () -> SWERVE.isAtPosition(targetPose)
                )
        );
    }

}
