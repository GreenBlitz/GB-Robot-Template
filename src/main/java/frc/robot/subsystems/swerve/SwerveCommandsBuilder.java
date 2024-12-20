package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.swerve.module.ModuleUtils;
import frc.robot.subsystems.swerve.module.Modules;
import frc.robot.subsystems.swerve.swervestatehelpers.RotateAxis;
import frc.utils.auto.PathPlannerUtils;
import frc.utils.calibration.swervecalibration.WheelRadiusCharacterization;
import frc.utils.calibration.sysid.SysIdCalibrator;
import frc.utils.utilcommands.InitExecuteCommand;

import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SwerveCommandsBuilder {

	private final Swerve swerve;
	private final Modules modules;
	private final SysIdCalibrator steerCalibrator;
	private final SysIdCalibrator driveCalibrator;

	public SwerveCommandsBuilder(Swerve swerve) {
		this.swerve = swerve;
		this.modules = swerve.getModules();
		this.steerCalibrator = new SysIdCalibrator(
			modules.getModule(ModuleUtils.ModulePosition.FRONT_LEFT).getSteerSysIdConfigInfo(),
			swerve,
			modules.getModule(ModuleUtils.ModulePosition.FRONT_LEFT)::setSteerVoltage
		);
		this.driveCalibrator = new SysIdCalibrator(
			modules.getModule(ModuleUtils.ModulePosition.FRONT_LEFT).getDriveSysIdConfigInfo(),
			swerve,
			modules::setDrivesVoltage
		);
	}

	public Command steerCalibration(boolean isQuasistatic, SysIdRoutine.Direction direction) {
		return steerCalibrator.getSysIdCommand(isQuasistatic, direction).withName("Steer calibration");
	}

	public Command driveCalibration(boolean isQuasistatic, SysIdRoutine.Direction direction) {
		Command sysIdCommand = driveCalibrator.getSysIdCommand(isQuasistatic, direction);
		sysIdCommand.getRequirements().clear();

		return new SequentialCommandGroup(
			pointWheels(new Rotation2d(), false).until(
				() -> modules.isSteersAtTargetPositions(
					SwerveConstants.CALIBRATION_MODULE_ANGLE_TOLERANCE,
					SwerveConstants.CALIBRATION_MODULE_ANGLE_VELOCITY_PER_SECOND_DEADBAND
				)
			),
			new ParallelDeadlineGroup(sysIdCommand, pointWheels(new Rotation2d(), false))
		).withName("Drive calibration");
	}


	public Command pointWheels(Rotation2d targetSteerPosition, boolean optimize) {
		return new RunCommand(() -> modules.pointWheels(targetSteerPosition, optimize), swerve)
			.withName("Point wheels to: " + targetSteerPosition);
	}

	public Command pointWheelsInCircle() {
		return new RunCommand(modules::pointWheelsInCircle, swerve).withName("Point wheels in circle");
	}

	public Command pointWheelsInX() {
		return new RunCommand(modules::pointWheelsInX, swerve).withName("Point wheels in X");
	}


	public Command setTargetModuleStates(Supplier<SwerveModuleState[]> statesSupplier, boolean isClosedLoop) {
		return new RunCommand(() -> modules.setTargetStates(statesSupplier.get(), isClosedLoop), swerve).withName("Set states by " + "supplier");
	}


	public Command wheelRadiusCalibration() {
		return new SequentialCommandGroup(
			pointWheelsInCircle().until(
				() -> swerve.getModules()
					.isSteersAtTargetPositions(
						SwerveConstants.CALIBRATION_MODULE_ANGLE_TOLERANCE,
						SwerveConstants.CALIBRATION_MODULE_ANGLE_VELOCITY_PER_SECOND_DEADBAND
					)
			),
			new WheelRadiusCharacterization(
				swerve,
				swerve.getDriveRadiusMeters(),
				SwerveConstants.WHEEL_RADIUS_CALIBRATION_VELOCITY_PER_SECOND,
				swerve.getModules()::getDrivesPositions,
				swerve::getAbsoluteHeading,
				rotationsPerSecond -> swerve.driveByState(new ChassisSpeeds(0, 0, rotationsPerSecond.getRadians()), SwerveState.DEFAULT_DRIVE),
				swerve.getModules()::stop
			)
		).withName("Wheel radius calibration");
	}


	public Command turnToHeading(Rotation2d targetHeading) {
		return turnToHeading(targetHeading, RotateAxis.MIDDLE_OF_ROBOT);
	}

	public Command turnToHeading(Rotation2d targetHeading, RotateAxis rotateAxis) {
		return new InitExecuteCommand(
			swerve::resetPIDControllers,
			() -> swerve.turnToHeading(targetHeading, SwerveState.DEFAULT_DRIVE.withRotateAxis(rotateAxis)),
			swerve
		).withName("Rotate around " + rotateAxis.name() + " to " + targetHeading);
	}


	public Command drive(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotationSupplier) {
		return driveState(xSupplier, ySupplier, rotationSupplier, SwerveState.DEFAULT_DRIVE).withName("Default drive");
	}

	public Command driveState(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotationSupplier, Supplier<SwerveState> state) {
		return new DeferredCommand(() -> driveState(xSupplier, ySupplier, rotationSupplier, state.get()), Set.of(swerve))
			.withName("Drive with supplier state");
	}

	public Command driveState(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rotationSupplier, SwerveState state) {
		return new InitExecuteCommand(
			swerve::resetPIDControllers,
			() -> swerve.driveByState(xSupplier.getAsDouble(), ySupplier.getAsDouble(), rotationSupplier.getAsDouble(), state),
			swerve
		).withName("Drive with state");
	}


	public Command driveToPose(Supplier<Pose2d> currentPose, Supplier<Pose2d> targetPose) {
		return new DeferredCommand(
			() -> new SequentialCommandGroup(pathToPose(currentPose.get(), targetPose.get()), pidToPose(currentPose, targetPose.get())),
			Set.of(swerve)
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

		return new SequentialCommandGroup(new InstantCommand(swerve::resetPIDControllers, swerve), pathFollowingCommand)
			.withName("Path to pose: " + targetPose);
	}

	private Command pidToPose(Supplier<Pose2d> currentPose, Pose2d targetPose) {
		return new InitExecuteCommand(swerve::resetPIDControllers, () -> swerve.pidToPose(currentPose.get(), targetPose), swerve)
			.withName("PID to pose: " + targetPose);
	}

}
