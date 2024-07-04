package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.modules.ModuleUtils;
import frc.robot.subsystems.swerve.swervestatehelpers.AimAssist;
import frc.robot.subsystems.swerve.swervestatehelpers.DriveSpeed;
import frc.robot.subsystems.swerve.swervestatehelpers.RotateAxis;
import frc.utils.calibration.swervecalibration.WheelRadiusCharacterization;
import frc.utils.calibration.sysid.SysIdCalibrator;
import frc.utils.pathplannerutils.PathPlannerUtils;
import frc.utils.utilcommands.InitExecuteCommand;

import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SwerveCommands {

    private static final Swerve SWERVE = RobotContainer.SWERVE;

    private static final SysIdCalibrator STEER_CALIBRATOR = new SysIdCalibrator(
            true,
            SWERVE,
            voltage -> SWERVE.runModuleSteerByVoltage(ModuleUtils.ModuleName.FRONT_LEFT, voltage),
            SwerveConstants.STEER_SYSID_CALIBRATION_VOLTAGE_STEP,
            SwerveConstants.STEER_SYSID_CALIBRATION_RAMP_RATE
    );

    private static final SysIdCalibrator DRIVE_CALIBRATOR = new SysIdCalibrator(
            true,
            SWERVE,
            SWERVE::runModulesDriveByVoltage,
            SwerveConstants.DRIVE_SYSID_CALIBRATION_VOLTAGE_STEP,
            SwerveConstants.DRIVE_SYSID_CALIBRATION_RAMP_RATE
    );

    public static Command steerCalibration(boolean isQuasistatic, SysIdRoutine.Direction direction) {
        Command command = STEER_CALIBRATOR.getSysIdCommand(isQuasistatic, direction);
        command.setName("Steer Calibration");
        return command;
    }

    public static Command driveCalibration(boolean isQuasistatic, SysIdRoutine.Direction direction) {
        Command sysIdCommand = DRIVE_CALIBRATOR.getSysIdCommand(isQuasistatic, direction);
        sysIdCommand.getRequirements().clear();

        Command command = new SequentialCommandGroup(
                pointWheels(new Rotation2d(), false),
                new ParallelDeadlineGroup(
                        sysIdCommand,
                        pointWheels(new Rotation2d(), false).repeatedly()
                )
        );
        command.setName("Drive Calibration");
        return command;
    }

    public static Command wheelRadiusCalibration() {
        Command command = new SequentialCommandGroup(
                pointWheelsInCircle(),
                new WheelRadiusCharacterization(
                        SWERVE,
                        SwerveConstants.DRIVE_RADIUS_METERS,
                        SwerveConstants.WHEEL_RADIUS_CALIBRATION_VELOCITY,
                        SWERVE::getModulesDriveDistances,
                        SWERVE::getAbsoluteHeading,
                        SWERVE::runWheelRadiusCharacterization,
                        SWERVE::stop
                )
        );
        command.setName("Wheel Radius Calibration");
        return command;
    }

    public static Command pointWheelsInX() {
        Command command = new FunctionalCommand(
                () -> {},
                SWERVE::pointWheelsInX,
                interrupted -> {},
                SWERVE::isModulesAtStates,
                SWERVE
        );
        command.setName("Point Wheels In X");
        return command;
    }

    public static Command pointWheelsInCircle() {
        Command command = new FunctionalCommand(
                () -> {},
                SWERVE::pointWheelsInCircle,
                interrupted -> {},
                SWERVE::isModulesAtStates,
                SWERVE
        );
        command.setName("Point Wheels In Circle");
        return command;
    }

    public static Command pointWheels(Rotation2d wheelsAngle, boolean optimize) {
        Command command = new FunctionalCommand(
                () -> {},
                () -> SWERVE.pointWheels(wheelsAngle, optimize),
                interrupted -> {},
                SWERVE::isModulesAtStates,
                SWERVE
        );
        command.setName("Point Wheels");
        return command;
    }

    public static Command driveSlow(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier) {
        Command command = new InitExecuteCommand(
                () -> SWERVE.initializeDrive(SwerveState.DEFAULT_DRIVE.withDriveSpeed(DriveSpeed.SLOW)),
                () -> SWERVE.drive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSupplier.getAsDouble()),
                SWERVE
        );
        command.setName("Slow Drive");
        return command;
    }

    public static Command driveWithAimAssist(
            DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier,
            AimAssist aimAssist
    ) {
        Command command = new InitExecuteCommand(
                () -> SWERVE.initializeDrive(SwerveState.DEFAULT_DRIVE.withAimAssist(aimAssist)),
                () -> SWERVE.drive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSupplier.getAsDouble()),
                SWERVE
        );
        command.setName("Rotate to Speaker");
        return command;
    }

    public static Command rotateToAngle(Rotation2d targetAngle) {
        return rotateToAngle(targetAngle, RotateAxis.MIDDLE_OF_ROBOT);
    }

    public static Command rotateToAngle(Rotation2d targetAngle, RotateAxis rotateAxis) {
        Command command = new FunctionalCommand(
                () -> SWERVE.initializeDrive(SwerveState.DEFAULT_DRIVE.withRotateAxis(rotateAxis)),
                () -> SWERVE.rotateToAngle(targetAngle),
                interrupted -> {},
                () -> SWERVE.isAtAngle(targetAngle),
                SWERVE
        );
        command.setName("Rotate Around " + rotateAxis.name() + "To " + targetAngle.getDegrees());
        return command;
    }


    public static Command driveAroundWheel(
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

    public static Command selfDrive(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier) {
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
    public static Command drive(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier) {
        Command defaultFieldRelative = new InitExecuteCommand(
                () -> SWERVE.initializeDrive(SwerveState.DEFAULT_DRIVE),
                () -> SWERVE.drive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSupplier.getAsDouble()),
                SWERVE
        );
        defaultFieldRelative.setName("Default - Field Relative");
        return defaultFieldRelative;
    }

    public static Command driveToPose(Supplier<Pose2d> targetPose, PathConstraints constraints) {
        Command command = new DeferredCommand(() -> driveToPose(targetPose.get(), constraints), Set.of(SWERVE));
        command.setName("Drive to " + targetPose.get());
        return command;
    }

    private static Command driveToPose(Pose2d targetPose, PathConstraints constraints) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> SWERVE.initializeDrive(SwerveState.DEFAULT_PATH_PLANNER)),
                pathToPose(targetPose, constraints),
                new InstantCommand(() -> SWERVE.initializeDrive(SwerveState.DEFAULT_DRIVE)),
                pidToPose(targetPose)
        );
    }

    private static Command pathToPose(Pose2d targetBluePose, PathConstraints pathConstraints) {
        Pose2d currentBluePose = RobotContainer.POSE_ESTIMATOR.getCurrentPose();

        double distanceFromTarget = currentBluePose.getTranslation().getDistance(targetBluePose.getTranslation());
        if (distanceFromTarget < SwerveConstants.CLOSE_TO_TARGET_POSITION_DEADBAND_METERS) {
            return PathPlannerUtils.createOnTheFlyPathCommand(currentBluePose, targetBluePose, pathConstraints);
        }
        return AutoBuilder.pathfindToPose(targetBluePose, pathConstraints);
    }

    private static Command pidToPose(Pose2d targetPose) {
        return new SequentialCommandGroup(
                new InstantCommand(SWERVE::resetRotationController),
                new RunCommand(() -> SWERVE.pidToPose(targetPose)).until(() -> SWERVE.isAtPosition(targetPose))
        );
    }

}
