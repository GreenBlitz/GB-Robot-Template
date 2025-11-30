package frc.robot.statemachine.shooterStateHandler;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.constants.turret.TurretConstants;
import frc.robot.subsystems.flywheel.FlyWheel;
import frc.utils.InterpolationMap;
import org.littletonrobotics.junction.Logger;

import java.util.Map;
import java.util.function.Supplier;

public class ShooterStateHandler {

	private final Arm turret;
	private final Arm hood;
	private final FlyWheel flyWheel;
	private ShooterState currentState;
	private final String logPath;
	private final Supplier<Double> distanceFromTower;

	public static final InterpolationMap<Double, Rotation2d> HOOD_INTERPOLATION_MAP = new InterpolationMap<Double, Rotation2d>(
		InverseInterpolator.forDouble(),
		InterpolationMap.interpolatorForRotation2d(),
		Map.of()
	);

	public static final InterpolationMap<Double, Rotation2d> FLYWHEEL_INTERPOLATION_MAP = new InterpolationMap<Double, Rotation2d>(
		InverseInterpolator.forDouble(),
		InterpolationMap.interpolatorForRotation2d(),
		Map.of()
	);

	public ShooterStateHandler(Arm turret, Arm hood, FlyWheel flyWheel, Supplier<Double> distanceFromTower, String logPath) {
		this.turret = turret;
		this.hood = hood;
		this.flyWheel = flyWheel;
		this.distanceFromTower = distanceFromTower;
		this.logPath = logPath;
	}

	public ShooterState getCurrentState() {
		return currentState;
	}

	public static Rotation2d hoodInterpolation(Supplier<Double> distanceFromTower) {
		return HOOD_INTERPOLATION_MAP.get(distanceFromTower.get());
	}

	public static Rotation2d flywheelInterpolation(Supplier<Double> distanceFromTower) {
		return FLYWHEEL_INTERPOLATION_MAP.get(distanceFromTower.get());
	}

	public Command setState(ShooterState shooterState) {
		return new ParallelCommandGroup(new InstantCommand(() -> currentState = shooterState), switch (shooterState) {
			case STAY_IN_PLACE -> stayInPlace();
			case IDLE -> idle();
			case SHOOT -> shoot();
		});
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
			turret.getCommandsBuilder().stayInPlace(),
			hood.getCommandsBuilder().setTargetPosition(hoodInterpolation(distanceFromTower)),
			flyWheel.getCommandBuilder().setTargetVelocity(ShooterConstants.DEFAULT_FLYWHEEL_ROTATIONS_PER_SECOND)
		);
	}

	private Command shoot() {
		return new ParallelCommandGroup(
			turret.getCommandsBuilder().stayInPlace(),
			hood.getCommandsBuilder().setTargetPosition(hoodInterpolation(distanceFromTower)),
			flyWheel.getCommandBuilder().setTargetVelocity(flywheelInterpolation(distanceFromTower))
		);
	}

	private Rotation2d getLookAtTowerAngleForTurret(Translation2d target, Pose2d robotPose) {
		double finalTargetRadians = (robotPose.getRotation().getRadians()
			+ Math.atan2(Math.abs(target.getY() - robotPose.getY()), Math.abs(target.getX() - robotPose.getX()))) % 360;
		return Rotation2d.fromRadians(finalTargetRadians);
	}

	public Command lookAtTower(Translation2d target, Pose2d robotPose) {
		Rotation2d finalLocation = getLookAtTowerAngleForTurret(target, robotPose);
		if (Math.abs(finalLocation.getDegrees() - turret.getPosition().getDegrees()) > 30 || Math.abs(ShooterConstants.SCREW_LOCATION.getDegrees() - turret.getPosition().getDegrees()) > 30) {
			return new ParallelCommandGroup(
				turret.getCommandsBuilder().setTargetPosition(finalLocation),
				new InstantCommand(() -> Logger.recordOutput(logPath + "/TurretTarget", finalLocation))
			);
		}
		return new ParallelCommandGroup(
			turret.getCommandsBuilder().stayInPlace(),
			new InstantCommand(() -> Logger.recordOutput(logPath + "/TurretTarget", finalLocation))
		);
	}

	public void Logger() {
		Logger.recordOutput(logPath + "/CurrentState", currentState);
	}

}
