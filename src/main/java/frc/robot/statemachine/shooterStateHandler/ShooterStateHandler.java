package frc.robot.statemachine.shooterStateHandler;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command;
import frc.constants.MathConstants;
import frc.constants.field.Tower;
import frc.robot.statemachine.ScoringHelpers;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.flywheel.FlyWheel;
import frc.utils.math.FieldMath;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class ShooterStateHandler {

	private final Arm turret;
	private final Arm hood;
	private final FlyWheel flyWheel;
	private final Supplier<Pose2d> robotPose;
	private ShooterState currentState;

	public ShooterStateHandler(Arm turret, Arm hood, FlyWheel flyWheel, Supplier<Pose2d> robotPose) {
		this.turret = turret;
		this.hood = hood;
		this.flyWheel = flyWheel;
		this.robotPose = robotPose;
		this.currentState = ShooterState.STAY_IN_PLACE;
	}

	public static Supplier<Rotation2d> hoodInterpolation(Supplier<Double> distanceFromTower) {
		return () -> ShooterConstants.HOOD_INTERPOLATION_MAP.get(distanceFromTower.get());
	}

	public static Supplier<Rotation2d> flywheelInterpolation(Supplier<Double> distanceFromTower) {
		return () -> ShooterConstants.FLYWHEEL_INTERPOLATION_MAP.get(distanceFromTower.get());
	}

	public ShooterState getCurrentState() {
		return currentState;
	}

	public Command setState(ShooterState shooterState) {
		Command command = switch (shooterState) {
			case STAY_IN_PLACE -> stayInPlace();
			case IDLE -> idle();
			case SHOOT -> shoot();
			case CALIBRATION -> calibration();
		};
		return new ParallelCommandGroup(
			new InstantCommand(() -> Logger.recordOutput(ShooterConstants.LOG_PATH + "/currentState", shooterState.name())),
			new InstantCommand(() -> currentState = shooterState),
			command
		);
	}

	private Command stayInPlace() {
		return new ParallelCommandGroup(
			turret.getCommandsBuilder().stayInPlace(),
			hood.getCommandsBuilder().stayInPlace(),
			flyWheel.getCommandBuilder().stop()
		);
	}

	private Command idle() {
		return new ParallelCommandGroup(
			aimAtTower(ScoringHelpers.getClosestTower(robotPose.get()).getTower(), robotPose.get()),
			hood.getCommandsBuilder()
				.setTargetPosition(hoodInterpolation(() -> getDistanceFromTower(ScoringHelpers.getClosestTower(robotPose.get())))),
			flyWheel.getCommandBuilder().setTargetVelocity(ShooterConstants.DEFAULT_FLYWHEEL_ROTATIONS_PER_SECOND)
		);
	}

	private Command shoot() {
		return new ParallelCommandGroup(
			aimAtTower(ScoringHelpers.getClosestTower(robotPose.get()).getTower(), robotPose.get()),
			hood.getCommandsBuilder()
				.setTargetPosition(hoodInterpolation(() -> getDistanceFromTower(ScoringHelpers.getClosestTower(robotPose.get())))),
			flyWheel.getCommandBuilder()
				.setVelocityAsSupplier(flywheelInterpolation(() -> getDistanceFromTower(ScoringHelpers.getClosestTower(robotPose.get()))))
		);
	}

	private Rotation2d getRobotRelativeLookAtTowerAngleForTurret(Translation2d target, Pose2d robotTranslation) {
		return getNormalizedAngle(FieldMath.getRelativeTranslation(robotTranslation, target).getAngle());
	}

	public double getDistanceFromTower(Tower tower) {
		return tower.getTower().getDistance(robotPose.get().getTranslation());
	}

	private boolean isTurretMoveIllegal(Rotation2d targetRobotRelative) {
		return Math.abs(targetRobotRelative.getDegrees() - turret.getPosition().getDegrees())
			< MathConstants.FULL_CIRCLE.getDegrees() - ShooterConstants.MAX_DISTANCE_FROM_SCREW_NOT_TO_ROTATE.getDegrees();
	}

	private static Rotation2d getNormalizedAngle(Rotation2d angle) {
		double degrees = angle.getDegrees() % MathConstants.FULL_CIRCLE.getDegrees();
		while (degrees < 0)
			degrees += MathConstants.FULL_CIRCLE.getDegrees();
		return Rotation2d.fromDegrees(degrees);
	}

	public Command aimAtTower(Translation2d target, Pose2d robotPose) {
		return new ConditionalCommand(
			new ParallelCommandGroup(
				turret.getCommandsBuilder().setTargetPosition(getRobotRelativeLookAtTowerAngleForTurret(target, robotPose)),
				new InstantCommand(() -> Logger.recordOutput(ShooterConstants.LOG_PATH + "/IsTurretGoingToPosition", true))
			),
			new ParallelCommandGroup(
				turret.getCommandsBuilder().stayInPlace(),
				new InstantCommand(() -> Logger.recordOutput(ShooterConstants.LOG_PATH + "/IsTurretGoingToPosition", false))
			),
			() -> isTurretMoveIllegal(getRobotRelativeLookAtTowerAngleForTurret(target, robotPose))
		);
	}

	public void Log() {
		Logger.recordOutput(ShooterConstants.LOG_PATH + "/CurrentState", currentState);
	}

	private Command calibration() {
		return new ParallelCommandGroup(
			turret.getCommandsBuilder().setTargetPosition(ShooterConstants.turretCalibrationAngle::get),
			hood.getCommandsBuilder().setTargetPosition(ShooterConstants.hoodCalibrationAngle::get),
			flyWheel.getCommandBuilder().setVelocityAsSupplier(ShooterConstants.flywheelCalibrationRotations::get)
		);
	}

}
