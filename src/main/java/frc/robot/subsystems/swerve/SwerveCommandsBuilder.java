package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.autonomous.AutonomousConstants;
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
			modules::setDrivesVoltage
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


	public Command driveToPose(Supplier<Pose2d> currentPose, Supplier<Pose2d> targetPose) {
		return new DeferredCommand(
			() -> new SequentialCommandGroup(pathToPose(currentPose.get(), targetPose.get()), pidToPose(currentPose, targetPose.get())),
			Set.of(swerve)
		);
	}
	
	public Command driveToPose(
			Supplier<Pose2d> currentPose,
			Supplier<Pose2d> targetPose,
			Supplier<ChassisSpeeds> chassisSpeedsSupplier,
			ProfiledPIDController xMetersProfiledPIDController,
			ProfiledPIDController yMetersProfiledPIDController,
			ProfiledPIDController thetaDegreesProfiledPIDController
	){
		return swerve.asSubsystemCommand(
				new DeferredCommand(
						() -> new FunctionalCommand(
								() -> {
									xMetersProfiledPIDController.reset(currentPose.get().getX(),chassisSpeedsSupplier.get().vxMetersPerSecond);
									yMetersProfiledPIDController.reset(currentPose.get().getY(),chassisSpeedsSupplier.get().vyMetersPerSecond);
									thetaDegreesProfiledPIDController.reset(currentPose.get().getRotation().getDegrees(), chassisSpeedsSupplier.get().omegaRadiansPerSecond);
									
									xMetersProfiledPIDController.setGoal(targetPose.get().getX());
									yMetersProfiledPIDController.setGoal(targetPose.get().getY());
									thetaDegreesProfiledPIDController.setGoal(targetPose.get().getRotation().getDegrees());
								},
								() -> {
									swerve.driveByState(
											new ChassisSpeeds(
													xMetersProfiledPIDController.calculate(currentPose.get().getX()),
													yMetersProfiledPIDController.calculate(currentPose.get().getY()),
													Units.degreesToRadians(thetaDegreesProfiledPIDController.calculate(currentPose.get().getRotation().getDegrees()))
											),
											SwerveState.DEFAULT_DRIVE.withLoopMode(LoopMode.CLOSED)
									);
								},
								(interrupted) -> {},
								() -> false,
								swerve
						),
						Set.of(swerve)
				),
				"drive to pose by profiled pid: " + targetPose.get().toString()
		);
	}
	
	public Command pathToPose(Supplier<Pose2d> currentPose, Supplier<Pose2d> targetPose){
		return swerve.asSubsystemCommand(
				new DeferredCommand(
						() -> pathToPose(currentPose.get(), targetPose.get()),
						Set.of(swerve)
				),
				"path to pose: " + targetPose.get().toString()
		);
	}

	private Command pathToPose(Pose2d currentPose, Pose2d targetPose) {
		Command pathFollowingCommand;
		double distanceFromTarget = currentPose.getTranslation().getDistance(targetPose.getTranslation());
		if (distanceFromTarget < AutonomousConstants.CLOSE_TO_TARGET_POSITION_DEADBAND_METERS) {
			pathFollowingCommand = PathPlannerUtil.createPathOnTheFly(currentPose, targetPose, AutonomousConstants.REAL_TIME_CONSTRAINTS);
		} else {
			pathFollowingCommand = AutoBuilder.pathfindToPose(targetPose, AutonomousConstants.REAL_TIME_CONSTRAINTS);
		}

		return swerve.asSubsystemCommand(
			new SequentialCommandGroup(new InstantCommand(swerve::resetPIDControllers), pathFollowingCommand),
			"Path to pose: " + targetPose
		);
	}

	public Command pidToPose(Supplier<Pose2d> currentPose, Pose2d targetPose) {
		return swerve.asSubsystemCommand(
			new InitExecuteCommand(swerve::resetPIDControllers, () -> swerve.moveToPoseByPID(currentPose.get(), targetPose)),
			"PID to pose: " + targetPose
		);
	}

}
