package frc.robot.statemachine.shooterstatehandler;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.constants.MathConstants;
import frc.robot.statemachine.ScoringHelpers;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.constants.turret.TurretConstants;
import frc.robot.subsystems.flywheel.FlyWheel;
import frc.utils.math.FieldMath;
import frc.utils.math.ToleranceMath;
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
			aimAtTower(() -> ScoringHelpers.getClosestTower(robotPose.get()).getTower()),
			hood.getCommandsBuilder().setTargetPosition(hoodInterpolation(() -> ScoringHelpers.getDistanceFromClosestTower(robotPose.get()))),
			flyWheel.getCommandBuilder().setTargetVelocity(ShooterConstants.DEFAULT_FLYWHEEL_ROTATIONS_PER_SECOND)
		);
	}

	private Command shoot() {
		return new ParallelCommandGroup(
			aimAtTower(() -> ScoringHelpers.getClosestTower(robotPose.get()).getTower()),
			hood.getCommandsBuilder().setTargetPosition(hoodInterpolation(() -> ScoringHelpers.getDistanceFromClosestTower(robotPose.get()))),
			flyWheel.getCommandBuilder()
				.setVelocityAsSupplier(
					flywheelInterpolation(
						() -> ScoringHelpers.getDistanceFromTower(ScoringHelpers.getClosestTower(robotPose.get()), robotPose.get())
					)
				)
		);
	}

	public static Supplier<Rotation2d> getRobotRelativeLookAtTowerAngleForTurret(Translation2d target, Pose2d robotPose) {
		Supplier<Rotation2d> targetAngle = () -> (Rotation2d
			.fromRadians(MathUtil.angleModulus(FieldMath.getRelativeTranslation(robotPose, target).getAngle().getRadians())));
		Supplier<Rotation2d> finalTargetAngle = targetAngle;
		targetAngle = () -> Rotation2d.fromDegrees(
			finalTargetAngle.get().getDegrees() < 0
				? finalTargetAngle.get().getDegrees() + MathConstants.FULL_CIRCLE.getDegrees()
				: finalTargetAngle.get().getDegrees()
		);
		return targetAngle;
	}

	public static boolean isTurretMoveLegal(Supplier<Rotation2d> targetRobotRelative, Arm turret) {
		double screwMaxToleranceDegrees = Rotation2d
			.fromRadians(
				MathUtil.angleModulus(
					Rotation2d
						.fromDegrees(
							TurretConstants.MAX_POSITION.getDegrees() - ShooterConstants.MAX_DISTANCE_FROM_MAX_OR_MIN_POSITION_NOT_TO_ROTATE.getDegrees()
						)
						.getRadians()
				)
			)
			.getDegrees(),
			screwMinToleranceDegrees = Rotation2d
				.fromRadians(
					MathUtil.angleModulus(
						TurretConstants.MIN_POSITION.getRadians() + ShooterConstants.MAX_DISTANCE_FROM_MAX_OR_MIN_POSITION_NOT_TO_ROTATE.getRadians()
					)
				)
				.getDegrees();
		if (screwMinToleranceDegrees < 0)
			screwMinToleranceDegrees += MathConstants.FULL_CIRCLE.getDegrees();
		if (screwMaxToleranceDegrees < 0)
			screwMaxToleranceDegrees += MathConstants.FULL_CIRCLE.getDegrees();
		Logger.recordOutput(TurretConstants.LOG_PATH + "/maxScrewTolerance", screwMaxToleranceDegrees);
		Logger.recordOutput(TurretConstants.LOG_PATH + "/minScrewTolerance", screwMinToleranceDegrees);
		Logger.recordOutput(TurretConstants.LOG_PATH + "/Target", targetRobotRelative.get());
		return ((!(targetRobotRelative.get().getDegrees() > screwMaxToleranceDegrees
			&& turret.getPosition().getDegrees() < screwMinToleranceDegrees)
			&& !(targetRobotRelative.get().getDegrees() < screwMinToleranceDegrees
				&& turret.getPosition().getDegrees() > screwMaxToleranceDegrees)))
			&& (ToleranceMath.isInRange(
				targetRobotRelative.get().getDegrees(),
				TurretConstants.BACKWARDS_SOFTWARE_LIMIT.getDegrees(),
				TurretConstants.FORWARD_SOFTWARE_LIMIT.getDegrees()
			));
	}

	public Command aimAtTower(Supplier<Translation2d> target) {
		return new TurretAimAtTowerCommand(turret, target, robotPose);
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
