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
import frc.robot.subsystems.swerve.swervestatehelpers.DriveSpeed;
import frc.robot.subsystems.swerve.swervestatehelpers.RotateAxis;
import frc.utils.calibration.swervecalibration.WheelRadiusCharacterization;
import frc.utils.mirrorutils.MirrorablePose2d;
import frc.utils.mirrorutils.MirrorableRotation2d;
import frc.utils.pathplannerutils.PathPlannerUtils;
import frc.utils.utilcommands.InitExecuteCommand;

import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SwerveCommands {

    private static final Swerve SWERVE = RobotContainer.SWERVE;

    public static Command getWheelRadiusCalibrationCommand() {
        Command command = new WheelRadiusCharacterization(
                SWERVE,
                SwerveConstants.DRIVE_RADIUS_METERS,
                Rotation2d.fromRotations(0.5),
                SWERVE::getModulesDriveDistances,
                SWERVE::getAbsoluteHeading,
                SWERVE::runWheelRadiusCharacterization,
                SWERVE::stop
        );
        command.setName("Wheel Radius Calibration");
        return command;
    }

    public static Command getLockSwerveCommand() {
        Command command = new FunctionalCommand(
                () -> {},
                SWERVE::lockSwerve,
                inter -> {},
                SWERVE::isModulesAtStates,
                SWERVE
        );
        command.setName("Lock");
        return command;
    }

    public static Command getPointWheelsCommand(MirrorableRotation2d wheelsAngle) {
        Command command = new FunctionalCommand(
                () -> {},
                () -> SWERVE.pointWheels(wheelsAngle),
                inter -> {},
                SWERVE::isModulesAtStates,
                SWERVE
        );
        command.setName("Point Wheels");
        return command;
    }

    public static Command getDriveSlowCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier) {
        Command command = new InitExecuteCommand(
                () -> SWERVE.initializeDrive(SwerveState.DEFAULT_DRIVE.withDriveSpeed(DriveSpeed.SLOW)),
                () -> SWERVE.drive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSupplier.getAsDouble()),
                SWERVE
        );
        command.setName("Slow Drive");
        return command;
    }

    public static Command getRotateToSpeaker(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier) {
        Command command = new InitExecuteCommand(
                () -> SWERVE.initializeDrive(SwerveState.DEFAULT_DRIVE.withAimAssist(AimAssist.SPEAKER)),
                () -> SWERVE.drive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSupplier.getAsDouble()),
                SWERVE
        );
        command.setName("Rotate to Speaker");
        return command;
    }

    public static Command getRotateToAngleCommand(MirrorableRotation2d targetAngle) {
        Command command = new FunctionalCommand(
                () -> SWERVE.initializeDrive(SwerveState.DEFAULT_DRIVE),
                () -> SWERVE.rotateToAngle(targetAngle),
                interrupted -> {},
                () -> SWERVE.isAtAngle(targetAngle),
                SWERVE
        );
        command.setName("Rotate To " + targetAngle.get().getDegrees());
        return command;
    }

    public static Command getRotateToAngleCommand(MirrorableRotation2d targetAngle, RotateAxis rotateAxis) {
        Command command = new FunctionalCommand(
                () -> SWERVE.initializeDrive(SwerveState.DEFAULT_DRIVE.withRotateAxis(rotateAxis)),
                () -> SWERVE.rotateToAngle(targetAngle),
                interrupted -> {},
                () -> SWERVE.isAtAngle(targetAngle),
                SWERVE
        );
        command.setName("Rotate Around " + rotateAxis.name());
        return command;
    }


    public static Command getDriveAroundWheelCommand(
            DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier, Supplier<RotateAxis> rotateAxis
    ) {
        Command command = new InitExecuteCommand(
                () -> SWERVE.initializeDrive(SwerveState.DEFAULT_DRIVE.withRotateAxis(rotateAxis.get())),
                () -> SWERVE.drive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSupplier.getAsDouble()),
                SWERVE
        );
        command.setName("Drive Around " + rotateAxis.get().name());
        return command;
    }

    public static Command getSelfDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier) {
        Command command = new InitExecuteCommand(
                () -> SWERVE.initializeDrive(SwerveState.DEFAULT_PATH_PLANNER),
                () -> SWERVE.drive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSupplier.getAsDouble()),
                SWERVE
        );
        command.setName("Self Relative Drive");
        return command;
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
    public static Command getOpenLoopFieldRelativeDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier,
            DoubleSupplier thetaSupplier) {
        Command defaultFieldRelative = new InitExecuteCommand(
                () -> SWERVE.initializeDrive(SwerveState.DEFAULT_DRIVE),
                () -> SWERVE.drive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSupplier.getAsDouble()),
                SWERVE
        );
        defaultFieldRelative.setName("Default - Field Relative");
        return defaultFieldRelative;
    }

    public static Command debugCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier) {
        Command command = new InitExecuteCommand(
                () -> SWERVE.initializeDrive(
                        SwerveState.DEFAULT_DRIVE
                                .withRotateAxis(RotateAxis.FRONT_LEFT_MODULE)
                                .withAimAssist(AimAssist.SPEAKER)
                ),
                () -> SWERVE.drive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSupplier.getAsDouble()),
                SWERVE
        );
        command.setName("Debug");
        return command;
    }


    public static Command getDriveToPoseCommand(Supplier<MirrorablePose2d> targetPose, PathConstraints constraints) {
        Command command = new DeferredCommand(() -> getCurrentDriveToPoseCommand(targetPose.get(), constraints), Set.of(SWERVE));
        command.setName("Drive to " + targetPose.get());
        return command;
    }

    private static Command getCurrentDriveToPoseCommand(MirrorablePose2d targetPose, PathConstraints constraints) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> SWERVE.initializeDrive(SwerveState.DEFAULT_PATH_PLANNER)),
                getPathfindToPoseCommand(targetPose, constraints),
                new InstantCommand(() -> SWERVE.initializeDrive(SwerveState.DEFAULT_DRIVE)),
                getPIDToPoseCommand(targetPose)
        );
    }

    private static Command getPathfindToPoseCommand(MirrorablePose2d targetPose, PathConstraints pathConstraints) {
        Pose2d currentPose = RobotContainer.POSE_ESTIMATOR.getCurrentPose();
        Pose2d targetMirroredPose = targetPose.get();

        double distance = currentPose.getTranslation().getDistance(targetMirroredPose.getTranslation());
        Command command;
        if (distance < SwerveConstants.CLOSE_TO_TARGET_POSITION_DEADBAND_METERS) {
            command = PathPlannerUtils.createOnTheFlyPathCommand(currentPose, targetMirroredPose, pathConstraints);
        }
        else {
            command = AutoBuilder.pathfindToPose(targetMirroredPose, pathConstraints);
        }
        command.setName("Path Finding to " + targetPose.get());
        return command;
    }

    private static Command getPIDToPoseCommand(MirrorablePose2d targetPose) {
        Command command = new SequentialCommandGroup(
                new InstantCommand(SWERVE::resetRotationController),
                new RunCommand(() -> SWERVE.pidToPose(targetPose)).until(() -> SWERVE.isAtPosition(targetPose))
        );
        command.setName("Pid To " + targetPose.get());
        return command;
    }

}
