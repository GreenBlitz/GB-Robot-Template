package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.swerve.swervestatehelpers.RotateAxis;
import frc.utils.auto.PathPlannerUtils;
import frc.utils.calibration.swervecalibration.WheelRadiusCharacterization;
import frc.utils.utilcommands.InitExecuteCommand;

import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SwerveCommandsBuilder {

	private final Swerve swerve;

	public SwerveCommandsBuilder(Swerve swerve) {
		this.swerve = swerve;
	}

	private Command toSwerveCommand(Command command) {
		command.addRequirements(swerve, swerve.getModules());
		return command;
	}

	public Command wheelRadiusCalibration() {
		return toSwerveCommand(new SequentialCommandGroup(
			swerve.getModules()
				.getCommandsBuilder()
				.pointWheelsInCircle()
				.until(
					() -> swerve.getModules()
						.isSteersAtTargetPositions(
							SwerveConstants.CALIBRATION_MODULE_ANGLE_TOLERANCE,
							SwerveConstants.CALIBRATION_MODULE_ANGLE_VELOCITY_PER_SECOND_DEADBAND
						)
				),
			new WheelRadiusCharacterization(
				swerve,
				swerve.getConstants().driveRadiusMeters(),
				SwerveConstants.WHEEL_RADIUS_CALIBRATION_VELOCITY_PER_SECOND,
				swerve.getModules()::getDrivesPositions,
				swerve::getAbsoluteHeading,
				rotationsPerSecond -> swerve.driveByState(new ChassisSpeeds(0, 0, rotationsPerSecond.getRadians()), SwerveState.DEFAULT_DRIVE),
				swerve.getModules()::stop
			)
		)).withName("Wheel radius calibration");
	}


	public Command turnToHeading(Rotation2d targetHeading) {
		return turnToHeading(targetHeading, RotateAxis.MIDDLE_OF_ROBOT);
	}

	public Command turnToHeading(Rotation2d targetHeading, RotateAxis rotateAxis) {
		return toSwerveCommand(new InitExecuteCommand(
			swerve::resetPIDControllers,
			() -> swerve.turnToHeading(targetHeading, SwerveState.DEFAULT_DRIVE.withRotateAxis(rotateAxis))
		)).withName("Rotate around " + rotateAxis.name() + " to " + targetHeading);
	}


	public Command drive(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotationSupplier) {
		return driveState(xSupplier, ySupplier, rotationSupplier, SwerveState.DEFAULT_DRIVE).withName("Default drive");
	}

	public Command driveState(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotationSupplier, Supplier<SwerveState> state) {
		return toSwerveCommand(new DeferredCommand(() -> driveState(xSupplier, ySupplier, rotationSupplier, state.get()), Set.of(swerve)))
			.withName("Drive with supplier state");
	}

	public Command driveState(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotationSupplier, SwerveState state) {
		return toSwerveCommand(new InitExecuteCommand(
			swerve::resetPIDControllers,
			() -> swerve.driveByState(xSupplier.getAsDouble(), ySupplier.getAsDouble(), rotationSupplier.getAsDouble(), state)
		)).withName("Drive with state");
	}


	public Command driveToPose(Supplier<Pose2d> currentPose, Supplier<Pose2d> targetPose) {
		return new DeferredCommand(
			() -> new SequentialCommandGroup(pathToPose(currentPose.get(), targetPose.get()), pidToPose(currentPose, targetPose.get())),
			Set.of(swerve, swerve.getModules())
		).withName("Drive to pose");
	}

	private Command pathToPose(Pose2d currentPose, Pose2d targetPose) {
		Command pathFollowingCommand;
		double distanceFromTarget = currentPose.getTranslation().getDistance(targetPose.getTranslation());
		if (distanceFromTarget < SwerveConstants.CLOSE_TO_TARGET_POSITION_DEADBAND_METERS) {
			pathFollowingCommand = PathPlannerUtils.createPathOnTheFly(currentPose, targetPose, SwerveConstants.REAL_TIME_CONSTRAINTS);
		} else {
			pathFollowingCommand = AutoBuilder.pathfindToPose(targetPose, SwerveConstants.REAL_TIME_CONSTRAINTS);
		}

		return toSwerveCommand(new SequentialCommandGroup(new InstantCommand(swerve::resetPIDControllers), pathFollowingCommand)
			.withName("Path to pose: " + targetPose));
	}

	private Command pidToPose(Supplier<Pose2d> currentPose, Pose2d targetPose) {
		return toSwerveCommand(new InitExecuteCommand(swerve::resetPIDControllers, () -> swerve.pidToPose(currentPose.get(), targetPose)))
			.withName("PID to pose: " + targetPose);
	}

}
