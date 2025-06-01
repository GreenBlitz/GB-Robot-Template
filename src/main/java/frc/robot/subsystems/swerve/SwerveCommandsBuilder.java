package frc.robot.subsystems.swerve;

import com.pathplanner.lib.path.PathPlannerPath;
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
import frc.robot.autonomous.AutonomousConstants;
import frc.robot.autonomous.PathFollowingCommandsBuilder;
import frc.robot.subsystems.swerve.factories.modules.drive.KrakenX60DriveBuilder;
import frc.robot.subsystems.swerve.module.ModuleUtil;
import frc.robot.subsystems.swerve.module.Modules;
import frc.robot.subsystems.swerve.states.LoopMode;
import frc.robot.subsystems.swerve.states.RotateAxis;
import frc.robot.subsystems.swerve.states.SwerveState;
import frc.utils.auto.PathPlannerUtil;
import frc.utils.calibration.swervecalibration.maxvelocityacceleration.MaxVelocityAccelerationCharacterization;
import frc.utils.calibration.swervecalibration.maxvelocityacceleration.VelocityType;
import frc.utils.calibration.swervecalibration.wheelradius.WheelRadiusCharacterization;
import frc.utils.calibration.sysid.SysIdCalibrator;
import frc.utils.utilcommands.InitExecuteCommand;

import java.util.Set;
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
			modules.getModule(ModuleUtil.ModulePosition.FRONT_LEFT).getSteerSysIdConfigInfo(),
			swerve,
			modules.getModule(ModuleUtil.ModulePosition.FRONT_LEFT)::setSteerVoltage
		);
		this.driveCalibrator = new SysIdCalibrator(
			modules.getModule(ModuleUtil.ModulePosition.FRONT_LEFT).getDriveSysIdConfigInfo(),
			swerve,
			KrakenX60DriveBuilder.IS_CURRENT_CONTROL ? modules::setDrivesCurrent : modules::setDrivesVoltage
		);
	}

	public Command steerCalibration(boolean isQuasistatic, SysIdRoutine.Direction direction) {
		return swerve.asSubsystemCommand(steerCalibrator.getSysIdCommand(isQuasistatic, direction), "Steer calibration");
	}

	public Command driveCalibration(boolean isQuasistatic, SysIdRoutine.Direction direction) {
		Command sysIdCommand = driveCalibrator.getSysIdCommand(isQuasistatic, direction);
		sysIdCommand.getRequirements().clear();

		return swerve.asSubsystemCommand(
			new SequentialCommandGroup(
				pointWheels(new Rotation2d(), false).until(
					() -> modules.isSteerAtTargetPositions(
						SwerveConstants.CALIBRATION_MODULE_ANGLE_TOLERANCE,
						SwerveConstants.CALIBRATION_MODULE_ANGULAR_VELOCITY_PER_SECOND_DEADBAND
					)
				),
				new ParallelDeadlineGroup(sysIdCommand, pointWheels(new Rotation2d(), false))
			),
			"Drive calibration"
		);
	}


	public Command pointWheels(Rotation2d targetSteerPosition, boolean optimize) {
		return swerve.asSubsystemCommand(
			new RunCommand(() -> modules.pointWheels(targetSteerPosition, optimize)),
			"Point wheels to: " + targetSteerPosition
		);
	}

	public Command pointWheelsInCircle() {
		return swerve.asSubsystemCommand(new RunCommand(modules::pointWheelsInCircle), "Point wheels in circle");
	}

	public Command pointWheelsInX() {
		return swerve.asSubsystemCommand(new RunCommand(modules::pointWheelsInX), "Point wheels in X");
	}


	public Command setTargetModuleStates(Supplier<SwerveModuleState[]> statesSupplier, boolean isClosedLoop) {
		return swerve.asSubsystemCommand(
			new RunCommand(() -> modules.setTargetStates(statesSupplier.get(), isClosedLoop)),
			"Set states by " + "supplier"
		);
	}


	public Command wheelRadiusCalibration() {
		return swerve.asSubsystemCommand(
			new SequentialCommandGroup(
				pointWheelsInCircle().until(
					() -> swerve.getModules()
						.isSteerAtTargetPositions(
							SwerveConstants.CALIBRATION_MODULE_ANGLE_TOLERANCE,
							SwerveConstants.CALIBRATION_MODULE_ANGULAR_VELOCITY_PER_SECOND_DEADBAND
						)
				),
				new WheelRadiusCharacterization(
					swerve,
					swerve.getDriveRadiusMeters(),
					SwerveConstants.WHEEL_RADIUS_CALIBRATION_VELOCITY_PER_SECOND,
					swerve.getModules()::getDrivesPositions,
					swerve::getAbsoluteHeading,
					rotationsPerSecond -> swerve
						.driveByState(new ChassisSpeeds(0, 0, rotationsPerSecond.getRadians()), SwerveState.DEFAULT_DRIVE),
					swerve.getModules()::stop
				)
			),
			"Wheel radius calibration"
		);
	}

	public Command maxVelocityAccelerationCalibration(VelocityType velocityType) {
		return swerve.asSubsystemCommand(
			new MaxVelocityAccelerationCharacterization(
				swerve,
				powers -> swerve.driveByState(powers, SwerveState.DEFAULT_DRIVE.withLoopMode(LoopMode.OPEN)),
				velocityType
			),
			"Max " + velocityType + " Velocity Acceleration Calibration"
		);
	}


	public Command turnToHeading(Rotation2d targetHeading) {
		return turnToHeading(targetHeading, RotateAxis.MIDDLE_OF_CHASSIS);
	}

	public Command turnToHeading(Rotation2d targetHeading, RotateAxis rotateAxis) {
		return swerve.asSubsystemCommand(
			new InitExecuteCommand(
				swerve::resetPIDControllers,
				() -> swerve.turnToHeading(targetHeading, SwerveState.DEFAULT_DRIVE.withRotateAxis(rotateAxis))
			),
			"Rotate around " + rotateAxis.name() + " to " + targetHeading
		);
	}


	public Command drive(Supplier<ChassisPowers> powersSupplier) {
		return driveByState(powersSupplier, SwerveState.DEFAULT_DRIVE);
	}

	public Command driveByState(Supplier<ChassisPowers> powersSupplier, Supplier<SwerveState> state) {
		return swerve.asSubsystemCommand(
			new DeferredCommand(() -> driveByState(powersSupplier, state.get()), Set.of(swerve)),
			"Drive with supplier state"
		);
	}

	public Command driveByState(Supplier<ChassisPowers> powersSupplier, SwerveState state) {
		return swerve.asSubsystemCommand(
			new InitExecuteCommand(swerve::resetPIDControllers, () -> swerve.driveByState(powersSupplier.get(), state)),
			"Drive with state"
		);
	}

	public Command driveByDriversInputs(Supplier<SwerveState> state) {
		return new DeferredCommand(() -> driveByDriversInputs(state.get()), Set.of(swerve));
	}

	public Command driveByDriversInputs(SwerveState state) {
		return swerve.asSubsystemCommand(
			new InitExecuteCommand(swerve::resetPIDControllers, () -> swerve.driveByDriversTargetsPowers(state)),
			"Drive by drivers inputs with state"
		);
	}

	public Command resetTargetSpeeds() {
		return swerve.asSubsystemCommand(
			new InstantCommand(() -> swerve.driveByState(new ChassisSpeeds(), SwerveState.DEFAULT_DRIVE)),
			"ResetTargetSpeeds"
		);
	}

	public Command driveToPose(Supplier<Pose2d> currentPose, Supplier<Pose2d> targetPose) {
		return new DeferredCommand(
			() -> new SequentialCommandGroup(pathToPose(currentPose.get(), targetPose.get()), moveToPoseByPID(currentPose, targetPose.get())),
			Set.of(swerve)
		);
	}

	private Command pathToPose(Pose2d currentPose, Pose2d targetPose) {
		Command pathFollowingCommand;
		if (PathPlannerUtil.isRobotInPathfindingDeadband(currentPose, targetPose)) {
			pathFollowingCommand = PathPlannerUtil
				.createPathDuringRuntime(currentPose, targetPose, AutonomousConstants.getRealTimeConstraints(swerve));
		} else {
			pathFollowingCommand = PathFollowingCommandsBuilder.pathfindToPose(targetPose, AutonomousConstants.getRealTimeConstraints(swerve));
		}

		return swerve.asSubsystemCommand(
			new SequentialCommandGroup(new InstantCommand(swerve::resetPIDControllers), pathFollowingCommand),
			"Path to pose: " + targetPose
		);
	}

	public Command driveToPath(Supplier<Pose2d> currentPose, PathPlannerPath path, Pose2d targetPose) {
		return new DeferredCommand(
			() -> new SequentialCommandGroup(
				PathFollowingCommandsBuilder.followPathOrPathfindAndFollowPath(swerve, path, currentPose),
				moveToPoseByPID(currentPose, targetPose)
			),
			Set.of(swerve)
		);
	}

	public Command moveToPoseByPID(Supplier<Pose2d> currentPose, Pose2d targetPose) {
		return swerve.asSubsystemCommand(
			new InitExecuteCommand(swerve::resetPIDControllers, () -> swerve.moveToPoseByPID(currentPose.get(), targetPose)),
			"PID to pose: " + targetPose
		);
	}

	public Command moveToPoseByPID(Supplier<Pose2d> currentPose, Pose2d targetPose, SwerveState swerveState) {
		return swerve.asSubsystemCommand(
			new InitExecuteCommand(swerve::resetPIDControllers, () -> swerve.moveToPoseByPID(currentPose.get(), targetPose, swerveState)),
			"PID to pose: " + targetPose
		);
	}

}
