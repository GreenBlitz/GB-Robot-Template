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
import java.util.function.Function;
import java.util.function.Supplier;

public class SwerveCommandsBuilder {

    private final Swerve swerve;

    private final SysIdCalibrator steerCalibrator;
    private final SysIdCalibrator driveCalibrator;

    protected SwerveCommandsBuilder(Swerve swerve) {
        this.swerve = swerve;
        this.steerCalibrator = new SysIdCalibrator(
                true,
                swerve,
                voltage -> swerve.getModules().runModuleSteerByVoltage(ModuleUtils.ModuleName.FRONT_LEFT, voltage),
                SwerveConstants.STEER_SYSID_CALIBRATION_VOLTAGE_STEP,
                SwerveConstants.STEER_SYSID_CALIBRATION_RAMP_RATE
        );
        this.driveCalibrator = new SysIdCalibrator(
                true,
                swerve,
                swerve.getModules()::runModulesDriveByVoltage,
                SwerveConstants.DRIVE_SYSID_CALIBRATION_VOLTAGE_STEP,
                SwerveConstants.DRIVE_SYSID_CALIBRATION_RAMP_RATE
        );
    }


    public Command steerCalibration(boolean isQuasistatic, SysIdRoutine.Direction direction) {
        return steerCalibrator.getSysIdCommand(isQuasistatic, direction).withName("Steer Calibration");
    }

    public Command driveCalibration(boolean isQuasistatic, SysIdRoutine.Direction direction) {
        Command sysIdCommand = driveCalibrator.getSysIdCommand(isQuasistatic, direction);
        sysIdCommand.getRequirements().clear();

        return new SequentialCommandGroup(
                pointWheels(new Rotation2d(), false),
                new ParallelDeadlineGroup(sysIdCommand, pointWheels(new Rotation2d(), false).repeatedly())
        ).withName("Drive Calibration");
    }

    public Command wheelRadiusCalibration() {
        return new SequentialCommandGroup(
                pointWheelsInCircle(),
                new WheelRadiusCharacterization(
                        swerve,
                        SwerveConstants.DRIVE_RADIUS_METERS,
                        SwerveConstants.WHEEL_RADIUS_CALIBRATION_VELOCITY,
                        swerve.getModules()::getModulesDriveDistances,
                        swerve::getAbsoluteHeading,
                        swerve::runWheelRadiusCharacterization,
                        swerve.getModules()::stop
                )
        ).withName("Wheel Radius Calibration");
    }


    public Command pointWheelsInX() {
        return new FunctionalCommand(
                () -> {},
                () -> swerve.getModules().pointWheelsInX(SwerveState.DEFAULT_DRIVE.getLoopMode().isClosedLoop),
                interrupted -> {},
                swerve.getModules()::isModulesAtAngles,
                swerve
        ).withName("Point Wheels In X");
    }

    public Command pointWheelsInCircle() {
        return new FunctionalCommand(
                () -> {},
                swerve.getModules()::pointWheelsInCircle,
                interrupted -> {},
                swerve.getModules()::isModulesAtAngles,
                swerve
        ).withName("Point Wheels In Circle");
    }

    public Command pointWheels(Rotation2d wheelsAngle, boolean optimize) {
        return new FunctionalCommand(
                () -> {},
                () -> swerve.getModules().pointWheels(wheelsAngle, optimize),
                interrupted -> {},
                swerve.getModules()::isModulesAtStates,
                swerve
        ).withName("Point Wheels");
    }


    public Command rotateToAngle(Rotation2d targetAngle) {
        return rotateToAngle(targetAngle, RotateAxis.MIDDLE_OF_ROBOT);
    }

    public Command rotateToAngle(Rotation2d targetAngle, RotateAxis rotateAxis) {
        return new FunctionalCommand(
                swerve::resetPIDControllers,
                () -> swerve.rotateToAngle(targetAngle),
                interrupted -> {},
                () -> swerve.isAtAngle(targetAngle),
                swerve
        ).withName("Rotate Around " + rotateAxis.name() + " To " + targetAngle.getDegrees() + " Degrees");
    }


    public Command driveWithAimAssist(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier,
            AimAssist aimAssist) {
        return driveState(
                xSupplier, ySupplier, thetaSupplier,
                () -> SwerveState.DEFAULT_DRIVE.withAimAssist(aimAssist)
        ).withName("Drive with Aim Assist");
    }

    public Command driveAroundWheel(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier,
            Supplier<RotateAxis> rotateAxis) {
        return driveState(
                xSupplier, ySupplier, thetaSupplier,
                () -> SwerveState.DEFAULT_DRIVE.withRotateAxis(rotateAxis.get())
        ).withName("Drive Around Module");
    }

    public Command driveSlow(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier) {
        return driveState(xSupplier, ySupplier, thetaSupplier, SwerveState.DEFAULT_DRIVE.withDriveSpeed(DriveSpeed.SLOW)).withName(
                "Slow Drive");
    }

    public Command driveRobotRelative(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier) {
        return driveState(xSupplier, ySupplier, thetaSupplier, SwerveState.DEFAULT_PATH_PLANNER).withName("Robot Relative Drive");
    }

    public Command drive(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier) {
        return driveState(xSupplier, ySupplier, thetaSupplier, SwerveState.DEFAULT_DRIVE).withName("Default Drive");
    }

    private Command driveState(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier, SwerveState state) {
        return driveState(xSupplier, ySupplier, thetaSupplier, () -> state);
    }

    private Command driveState(
            DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier,
            Supplier<SwerveState> state
    ) {
        return new InitExecuteCommand(
                swerve::resetPIDControllers,
                () -> swerve.driveByState(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSupplier.getAsDouble(), state.get()),
                swerve
        );
    }


    public Command driveToPose(Supplier<Pose2d> currentPose, Supplier<Pose2d> targetPose, Function<Pose2d, Boolean> isAtPose) {
        return new DeferredCommand(
                () -> new SequentialCommandGroup(
                        pathToPose(currentPose.get(), targetPose.get()),
                        pidToPose(currentPose, targetPose.get(), isAtPose)
                ),
                Set.of(swerve)
        ).withName("Drive to Pose");
    }

    private Command pathToPose(Pose2d currentPose, Pose2d targetPose) {
        Command pathFollowingCommand;
        double distanceFromTarget = currentPose.getTranslation().getDistance(targetPose.getTranslation());
        if (distanceFromTarget < SwerveConstants.CLOSE_TO_TARGET_POSITION_DEADBAND_METERS) {
            pathFollowingCommand = PathPlannerUtils.createOnTheFlyPathCommand(
                    currentPose,
                    targetPose,
                    SwerveConstants.REAL_TIME_CONSTRAINTS
            );
        }
        else {
            pathFollowingCommand = AutoBuilder.pathfindToPose(targetPose, SwerveConstants.REAL_TIME_CONSTRAINTS);
        }

        return new SequentialCommandGroup(
                new InstantCommand(swerve::resetPIDControllers),
                pathFollowingCommand
        ).withName("Path to Pose: " + targetPose);
    }

    private Command pidToPose(Supplier<Pose2d> currentPose, Pose2d targetPose, Function<Pose2d, Boolean> isAtPose) {
        return new SequentialCommandGroup(
                new InstantCommand(swerve::resetPIDControllers),
                new RunCommand(() -> swerve.pidToPose(currentPose.get(), targetPose))
                        .until(() -> isAtPose.apply(targetPose))
        ).withName("PID to Pose: " + targetPose);
    }

}
