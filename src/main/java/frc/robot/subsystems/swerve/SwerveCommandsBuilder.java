package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.swerve.modules.ModuleUtils;
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

	public SwerveCommandsBuilder(Swerve swerve) {
		this.swerve = swerve;
		this.steerCalibrator = new SysIdCalibrator(
			true,
			swerve,
			voltage -> swerve.getModules().setSteersVoltage(ModuleUtils.ModulePosition.FRONT_LEFT, voltage),
			SwerveConstants.STEER_SYSID_CALIBRATION_VOLTAGE_STEP,
			SwerveConstants.STEER_SYSID_CALIBRATION_RAMP_RATE
		);
		this.driveCalibrator = new SysIdCalibrator(
			true,
			swerve,
			swerve.getModules()::setDrivesVoltage,
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
				swerve.getModules()::getDrivesAngles,
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
			swerve.getModules()::isAtTargetAngles,
			swerve
		).withName("Point Wheels In X");
	}

	public Command pointWheelsInCircle() {
		return new FunctionalCommand(
			() -> {},
			swerve.getModules()::pointWheelsInCircle,
			interrupted -> {},
			swerve.getModules()::isAtTargetAngles,
			swerve
		).withName("Point Wheels In Circle");
	}

	public Command pointWheels(Rotation2d wheelsAngle, boolean optimize) {
		return new FunctionalCommand(
			() -> {},
			() -> swerve.getModules().pointWheels(wheelsAngle, optimize),
			interrupted -> {},
			swerve.getModules()::isAtTargetStates,
			swerve
		).withName("Point Wheels");
	}


	public Command rotateToAngle(Rotation2d targetAngle) {
		return rotateToAngle(targetAngle, RotateAxis.MIDDLE_OF_ROBOT);
	}

	public Command rotateToAngle(Rotation2d targetAngle, RotateAxis rotateAxis) {
		return new FunctionalCommand(
			swerve::resetPIDControllers,
			() -> swerve.rotateToAngle(targetAngle, SwerveState.DEFAULT_DRIVE.withRotateAxis(rotateAxis)),
			interrupted -> {},
			() -> swerve.isAtAngle(targetAngle),
			swerve
		).withName("Rotate Around " + rotateAxis.name() + " To " + targetAngle.getDegrees() + " Degrees");
	}


	public Command drive(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier) {
		return driveState(xSupplier, ySupplier, thetaSupplier, SwerveState.DEFAULT_DRIVE).withName("Default Drive");
	}

	public Command driveState(
		DoubleSupplier xSupplier,
		DoubleSupplier ySupplier,
		DoubleSupplier thetaSupplier,
		Supplier<SwerveState> state
	) {
		return new DeferredCommand(() -> driveState(xSupplier, ySupplier, thetaSupplier, state.get()), Set.of(swerve))
			.withName("Drive With Supplier State");
	}

	public Command
		driveState(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier, SwerveState state) {
		return new InitExecuteCommand(
			swerve::resetPIDControllers,
			() -> swerve.driveByState(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSupplier.getAsDouble(), state),
			swerve
		).withName("Drive With State");
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
			pathFollowingCommand = PathPlannerUtils
				.createOnTheFlyPathCommand(currentPose, targetPose, SwerveConstants.REAL_TIME_CONSTRAINTS);
		} else {
			pathFollowingCommand = AutoBuilder.pathfindToPose(targetPose, SwerveConstants.REAL_TIME_CONSTRAINTS);
		}

		return new SequentialCommandGroup(new InstantCommand(swerve::resetPIDControllers), pathFollowingCommand)
			.withName("Path to Pose: " + targetPose);
	}

	private Command pidToPose(Supplier<Pose2d> currentPose, Pose2d targetPose, Function<Pose2d, Boolean> isAtPose) {
		return new FunctionalCommand(
			swerve::resetPIDControllers,
			() -> swerve.pidToPose(currentPose.get(), targetPose),
			interrupted -> {},
			() -> isAtPose.apply(targetPose),
			swerve
		).withName("PID to Pose: " + targetPose);
	}

}
