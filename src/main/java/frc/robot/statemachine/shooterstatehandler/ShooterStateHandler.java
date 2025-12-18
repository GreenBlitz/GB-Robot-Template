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

	public void Log() {
		Logger.recordOutput(ShooterConstants.LOG_PATH + "/CurrentState", currentState);
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
			aimAtTower(),
			hood.getCommandsBuilder().setTargetPosition(hoodInterpolation(() -> ScoringHelpers.getDistanceFromClosestTower(robotPose.get()))),
			flyWheel.getCommandBuilder().setTargetVelocity(ShooterConstants.DEFAULT_FLYWHEEL_ROTATIONS_PER_SECOND)
		);
	}

	private Command shoot() {
		return new ParallelCommandGroup(
			aimAtTower(),
			hood.getCommandsBuilder().setTargetPosition(hoodInterpolation(() -> ScoringHelpers.getDistanceFromClosestTower(robotPose.get()))),
			flyWheel.getCommandBuilder()
				.setVelocityAsSupplier(flywheelInterpolation(() -> ScoringHelpers.getDistanceFromClosestTower(robotPose.get())))
		);
	}

	private Command calibration() {
		return new ParallelCommandGroup(
			turret.getCommandsBuilder().setTargetPosition(ShooterConstants.turretCalibrationAngle::get),
			hood.getCommandsBuilder().setTargetPosition(ShooterConstants.hoodCalibrationAngle::get),
			flyWheel.getCommandBuilder().setVelocityAsSupplier(ShooterConstants.flywheelCalibrationRotations::get)
		);
	}

	public Command aimAtTower() {
		return new TurretAimAtTowerCommand(turret, robotPose);
	}

	public static Rotation2d getRobotRelativeLookAtTowerAngleForTurret(Translation2d target, Pose2d robotPose) {
		Rotation2d targetAngle = Rotation2d.fromRadians(FieldMath.getRelativeTranslation(robotPose, target).getAngle().getRadians());
		return Rotation2d
			.fromDegrees(MathUtil.inputModulus(targetAngle.getDegrees(), Rotation2d.kZero.getDegrees(), MathConstants.FULL_CIRCLE.getDegrees()));
	}

	public static boolean isTurretMoveLegal(Rotation2d targetRobotRelative, Arm turret) {
		boolean isTargetInMaxRange = !(targetRobotRelative.getDegrees() > TurretConstants.SCREW_MAX_RANGE_EDGE.getDegrees()
			&& turret.getPosition().getDegrees() < TurretConstants.SCREW_MIN_RANGE_EDGE.getDegrees());

		boolean isTargetInMinRange = !(targetRobotRelative.getDegrees() < TurretConstants.SCREW_MIN_RANGE_EDGE.getDegrees()
			&& turret.getPosition().getDegrees() > TurretConstants.SCREW_MAX_RANGE_EDGE.getDegrees());

		boolean isTargetBehindSoftwareLimits = ToleranceMath.isInRange(
			targetRobotRelative.getDegrees(),
			TurretConstants.BACKWARDS_SOFTWARE_LIMIT.getDegrees(),
			TurretConstants.FORWARD_SOFTWARE_LIMIT.getDegrees()
		);

		return isTargetInMaxRange && isTargetInMinRange && isTargetBehindSoftwareLimits;
	}

	public static Rotation2d getRangeEdge(Rotation2d angle, Rotation2d tolerance) {
		return Rotation2d.fromRadians(
			MathUtil
				.inputModulus(angle.getRadians() + tolerance.getRadians(), Rotation2d.kZero.getRadians(), MathConstants.FULL_CIRCLE.getRadians())
		);
	}

}
