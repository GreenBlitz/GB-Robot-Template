package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
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
import frc.robot.Robot;
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

    private static final Swerve swerve = Robot.swerve;

    private static final SysIdCalibrator STEER_CALIBRATOR = new SysIdCalibrator(// todo : maybe move place
            true,
            swerve,
            voltage -> swerve.runModuleSteerByVoltage(ModuleUtils.ModuleName.FRONT_LEFT, voltage),
            SwerveConstants.STEER_SYSID_CALIBRATION_VOLTAGE_STEP,
            SwerveConstants.STEER_SYSID_CALIBRATION_RAMP_RATE
    );

    private static final SysIdCalibrator DRIVE_CALIBRATOR = new SysIdCalibrator(// todo : maybe move place
            true,
            swerve,
            swerve::runModulesDriveByVoltage,
            SwerveConstants.DRIVE_SYSID_CALIBRATION_VOLTAGE_STEP,
            SwerveConstants.DRIVE_SYSID_CALIBRATION_RAMP_RATE
    );


    public static Command steerCalibration(boolean isQuasistatic, SysIdRoutine.Direction direction) {
        Command steerCalibration = STEER_CALIBRATOR.getSysIdCommand(isQuasistatic, direction);
        steerCalibration.setName("Steer Calibration");
        return steerCalibration;
    }

    public static Command driveCalibration(boolean isQuasistatic, SysIdRoutine.Direction direction) {
        Command sysIdCommand = DRIVE_CALIBRATOR.getSysIdCommand(isQuasistatic, direction);
        sysIdCommand.getRequirements().clear();

        Command driveCalibration = new SequentialCommandGroup(
                pointWheels(new Rotation2d(), false),
                new ParallelDeadlineGroup(
                        sysIdCommand,
                        pointWheels(new Rotation2d(), false).repeatedly()
                )
        );
        driveCalibration.setName("Drive Calibration");
        return driveCalibration;
    }

    public static Command wheelRadiusCalibration() {
        Command wheelRadiusCalibration = new SequentialCommandGroup(
                pointWheelsInCircle(),
                new WheelRadiusCharacterization(
                        swerve,
                        SwerveConstants.DRIVE_RADIUS_METERS,
                        SwerveConstants.WHEEL_RADIUS_CALIBRATION_VELOCITY,
                        swerve::getModulesDriveDistances,
                        swerve::getAbsoluteHeading,
                        swerve::runWheelRadiusCharacterization,
                        swerve::stop
                )
        );
        wheelRadiusCalibration.setName("Wheel Radius Calibration");
        return wheelRadiusCalibration;
    }


    public static Command pointWheelsInX() {
        Command pointWheelsInX = new FunctionalCommand(
                () -> {},
                swerve::pointWheelsInX,
                interrupted -> {},
                swerve::isModulesAtStates,
                swerve
        );
        pointWheelsInX.setName("Point Wheels In X");
        return pointWheelsInX;
    }

    public static Command pointWheelsInCircle() {
        Command pointWheelsInCircle = new FunctionalCommand(
                () -> {},
                swerve::pointWheelsInCircle,
                interrupted -> {},
                swerve::isModulesAtStates,
                swerve
        );
        pointWheelsInCircle.setName("Point Wheels In Circle");
        return pointWheelsInCircle;
    }

    public static Command pointWheels(Rotation2d wheelsAngle, boolean optimize) {
        Command pointWheels = new FunctionalCommand(
                () -> {},
                () -> swerve.pointWheels(wheelsAngle, optimize),
                interrupted -> {},
                swerve::isModulesAtStates,
                swerve
        );
        pointWheels.setName("Point Wheels");
        return pointWheels;
    }


    public static Command rotateToAngle(Rotation2d targetAngle) {
        return rotateToAngle(targetAngle, RotateAxis.MIDDLE_OF_ROBOT);
    }

    public static Command rotateToAngle(Rotation2d targetAngle, RotateAxis rotateAxis) {
        Command rotateToAngle = new FunctionalCommand(
                () -> swerve.initializeDrive(SwerveState.DEFAULT_DRIVE.withRotateAxis(rotateAxis)),
                () -> swerve.rotateToAngle(targetAngle),
                interrupted -> {},
                () -> Robot.poseEstimator.isAtAngle(targetAngle),
                swerve
        );
        rotateToAngle.setName("Rotate Around " + rotateAxis.name() + "To " + targetAngle.getDegrees());
        return rotateToAngle;
    }


    public static Command driveSlow(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier) {
        Command driveSlow = new InitExecuteCommand(
                () -> swerve.initializeDrive(SwerveState.DEFAULT_DRIVE.withDriveSpeed(DriveSpeed.SLOW)),
                () -> swerve.drive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSupplier.getAsDouble()),
                swerve
        );
        driveSlow.setName("Slow Drive");
        return driveSlow;
    }

    public static Command driveWithAimAssist(
            DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier, AimAssist aimAssist
    ) {
        Command driveWithAimAssist = new InitExecuteCommand(
                () -> swerve.initializeDrive(SwerveState.DEFAULT_DRIVE.withAimAssist(aimAssist)),
                () -> swerve.drive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSupplier.getAsDouble()),
                swerve
        );
        driveWithAimAssist.setName("Rotate to Speaker");
        return driveWithAimAssist;
    }

    public static Command driveAroundWheel(
            DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier, Supplier<RotateAxis> rotateAxis
    ) {
        Command driveAroundWheel = new InitExecuteCommand(
                () -> swerve.initializeDrive(SwerveState.DEFAULT_DRIVE.withRotateAxis(rotateAxis.get())),
                () -> swerve.drive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSupplier.getAsDouble()),
                swerve
        );
        driveAroundWheel.setName("Drive Around " + rotateAxis.get().name());
        return driveAroundWheel;
    }

    public static Command driveSelfRelative(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier) {
        Command driveSelfRelative = new InitExecuteCommand(
                () -> swerve.initializeDrive(SwerveState.DEFAULT_PATH_PLANNER),
                () -> swerve.drive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSupplier.getAsDouble()),
                swerve
        );
        driveSelfRelative.setName("Self Relative Drive");
        return driveSelfRelative;
    }

    public static Command drive(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier) {
        Command drive = new InitExecuteCommand(
                () -> swerve.initializeDrive(SwerveState.DEFAULT_DRIVE),
                () -> swerve.drive(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSupplier.getAsDouble()),
                swerve
        );
        drive.setName("Default Drive");
        return drive;
    }


    public static Command driveToPose(Supplier<Pose2d> targetPose) {
        Command driveToPose = new DeferredCommand(() -> driveToPose(targetPose.get()), Set.of(swerve));
        driveToPose.setName("Drive to " + targetPose.get());
        return driveToPose;
    }

    public static Command driveToPose(Pose2d targetPose) {
        Command driveToPose =  new SequentialCommandGroup(
                new InstantCommand(() -> swerve.initializeDrive(SwerveState.DEFAULT_PATH_PLANNER)),
                pathToPose(targetPose),
                new InstantCommand(() -> swerve.initializeDrive(SwerveState.DEFAULT_DRIVE)),
                pidToPose(targetPose)
        );
        driveToPose.setName("Drive to " + targetPose);
        return driveToPose;
    }

    private static Command pathToPose(Pose2d targetBluePose) {
        Pose2d currentBluePose = Robot.poseEstimator.getCurrentPose();

        double distanceFromTarget = currentBluePose.getTranslation().getDistance(targetBluePose.getTranslation());
        if (distanceFromTarget < SwerveConstants.CLOSE_TO_TARGET_POSITION_DEADBAND_METERS) {
            return PathPlannerUtils.createOnTheFlyPathCommand(currentBluePose, targetBluePose, SwerveConstants.REAL_TIME_CONSTRAINTS);
        }
        return AutoBuilder.pathfindToPose(targetBluePose, SwerveConstants.REAL_TIME_CONSTRAINTS);
    }

    private static Command pidToPose(Pose2d targetPose) {
        return new SequentialCommandGroup(
                new InstantCommand(swerve::resetRotationController),
                new RunCommand(() -> swerve.pidToPose(targetPose)).until(() -> Robot.poseEstimator.isAtPose(targetPose))
        );
    }

}
