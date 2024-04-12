package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.utils.allianceutils.AlliancePose2d;
import frc.utils.commands.InitExecuteCommand;

import java.util.List;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.RobotContainer.SWERVE;

public class SwerveCommands {

    /**
     * Creates a command that drives the swerve with the given powers, relative to the field's frame of reference, in closed
     * open mode.
     *
     * @param xSupplier     the target forwards power
     * @param ySupplier     the target leftwards power
     * @param thetaSupplier the target theta power, CCW+
     * @return the command
     */
    public static Command getOpenLoopFieldRelativeDriveCommand(
            DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier
    ) {
        return new InitExecuteCommand(() -> SWERVE.initializeDrive(false),
                () -> SWERVE.fieldRelativeDrive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSupplier.getAsDouble()),
                SWERVE
        );
    }

    /**
     * Creates a command that drives the swerve with the given powers, relative to the field's frame of reference, in closed
     * loop mode.
     *
     * @param xSupplier     the target forwards power
     * @param ySupplier     the target leftwards power
     * @param thetaSupplier the target theta power, CCW+
     * @return the command
     */
    public static Command getClosedLoopFieldRelativeDriveCommand(
            DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier
    ) {
        return new InitExecuteCommand(() -> SWERVE.initializeDrive(true),
                () -> SWERVE.fieldRelativeDrive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSupplier.getAsDouble()),
                SWERVE
        );
    }

    /**
     * Creates a command that drives the swerve with the given powers, and rotating around wheel instead of middle if robot,
     * relative to the field's frame of reference, in open loop mode.
     *
     * @param xSupplier     the target forwards power
     * @param ySupplier     the target leftwards power
     * @param thetaSupplier the target theta power, CCW+
     * @param moduleName    the wanted module to rotate around
     * @return the command
     */
    public static Command getOpenLoopFieldRelativeDriveWithRotateAroundWheelCommand(
            DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier, ModuleUtils.ModuleName moduleName
    ) {
        return new InitExecuteCommand(() -> SWERVE.initializeDrive(false),
                () -> SWERVE.fieldRelativeDriveRotateAroundModule(xSupplier.getAsDouble(),
                        ySupplier.getAsDouble(),
                        thetaSupplier.getAsDouble(),
                        moduleName
                ),
                SWERVE
        );
    }

    /**
     * Creates a command that drives the swerve with the given powers, and rotating around wheel instead of middle if robot,
     * relative to the field's frame of reference, in closed loop mode.
     *
     * @param xSupplier     the target forwards power
     * @param ySupplier     the target leftwards power
     * @param thetaSupplier the target theta power, CCW+
     * @param moduleName    the wanted module to rotate around
     * @return the command
     */
    public static Command getClosedLoopFieldRelativeDriveWithRotateAroundWheelCommand(
            DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier, ModuleUtils.ModuleName moduleName
    ) {
        return new InitExecuteCommand(() -> SWERVE.initializeDrive(true),
                () -> SWERVE.fieldRelativeDriveRotateAroundModule(xSupplier.getAsDouble(),
                        ySupplier.getAsDouble(),
                        thetaSupplier.getAsDouble(),
                        moduleName
                ),
                SWERVE
        );
    }

    /**
     * Creates a command that rotating the swerve to target angle by trapezoid profiled pid
     *
     * @param targetAngle -> the targetAngle in Rotation2d
     * @param moduleName  -> the module to turn around
     * @return the command
     */
    public static Command getRotateToAngleAroundWheelCommand(
            Rotation2d targetAngle, ModuleUtils.ModuleName moduleName
    ) {
        return new InstantCommand(SWERVE::resetRotationController).andThen(new RunCommand(() -> SWERVE.rotateToAngleAroundWheel(
                targetAngle,
                moduleName
        ))).until(() -> SWERVE.isAtAngle(targetAngle));
    }

    /**
     * Creates a command that rotating the swerve to target angle by trapezoid profiled pid
     *
     * @param targetAngle -> the targetAngle in Rotation2d
     * @return the command
     */
    public static Command getRotateToAngleCommand(Rotation2d targetAngle) {
        return new InstantCommand(SWERVE::resetRotationController).andThen(new RunCommand(() -> SWERVE.rotateToAngle(targetAngle))).until(
                () -> SWERVE.isAtAngle(targetAngle));
    }

    /**
     * Wrapper to "getCurrentDriveToPoseCommand()" that can handle suppliers
     *
     * @param targetPose -> the target positions as AlliancePose2d
     * @return the command
     */
    public static Command getDriveToPoseCommand(Supplier<AlliancePose2d> targetPose, PathConstraints constraints) {
        return new DeferredCommand(() -> getCurrentDriveToPoseCommand(targetPose.get(), constraints), Set.of(SWERVE));
    }

    /**
     * Creates a command that drives the swerve to a target position.
     * Doing follow path and then pid
     *
     * @param targetPose -> the target positions as AlliancePose2d
     * @return the command
     */
    private static Command getCurrentDriveToPoseCommand(AlliancePose2d targetPose, PathConstraints constraints) {
        return new SequentialCommandGroup(new InstantCommand(() -> SWERVE.initializeDrive(true)),
                getPathfindToPoseCommand(targetPose, constraints),
                getPIDToPoseCommand(targetPose)
        );
    }

    // Todo - add doc
    private static Command getPathfindToPoseCommand(AlliancePose2d targetPose, PathConstraints pathConstraints) {
        final Pose2d targetMirroredAlliancePose = targetPose.toMirroredAlliancePose();
        final Pose2d currentBluePose = RobotContainer.POSE_ESTIMATOR.getCurrentPose().toBlueAlliancePose();
        // TODO - understand if statement
        if (currentBluePose.getTranslation().getDistance(targetMirroredAlliancePose.getTranslation()) < SwerveConstants.CLOSE_TO_TARGET_POSITION_DEADBAND_METERS)
        // TODO - find difference between the two funcs
        {
            return createOnTheFlyPathCommand(targetMirroredAlliancePose, pathConstraints);
        }
        return AutoBuilder.pathfindToPose(targetMirroredAlliancePose, pathConstraints);
    }

    /**
     * Creates a command that drives the swerve to a target position using pid.
     * Mostly used after follow path to correct the ending positions after the path follow.
     *
     * @param targetPose -> the target positions as AlliancePose2d
     * @return the command
     */
    private static Command getPIDToPoseCommand(AlliancePose2d targetPose) {
        return new InstantCommand(SWERVE::resetRotationController).andThen(new RunCommand(() -> SWERVE.pidToPose(targetPose.toMirroredAlliancePose())).until(
                () -> SWERVE.isAtPosition(targetPose.toMirroredAlliancePose())));
    }

    // Todo - add doc
    private static Command createOnTheFlyPathCommand(Pose2d targetPose, PathConstraints constraints) {
        List<Translation2d> bezierPoints =
                PathPlannerPath.bezierFromPoses(RobotContainer.POSE_ESTIMATOR.getCurrentPose().toAlliancePose(),
                targetPose
        );

        PathPlannerPath path = new PathPlannerPath(bezierPoints, constraints, new GoalEndState(0, targetPose.getRotation()));

        path.preventFlipping = true;

        return AutoBuilder.followPath(path);
    }

}
