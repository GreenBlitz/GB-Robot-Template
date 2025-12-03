package frc.robot.statemachine.shooterStateHandler;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.flywheel.FlyWheel;
import frc.utils.math.FieldMath;
import frc.utils.pose.PoseUtil;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class ShooterStateHandler {

	private final Arm turret;
	private final Arm hood;
	private final FlyWheel flyWheel;
	private ShooterState currentState;
	private final Supplier<Double> distanceFromTower;

	public ShooterStateHandler(Arm turret, Arm hood, FlyWheel flyWheel, Supplier<Double> distanceFromTower) {
		this.turret = turret;
		this.hood = hood;
		this.flyWheel = flyWheel;
		this.distanceFromTower = distanceFromTower;
		this.currentState = ShooterState.STAY_IN_PLACE;
	}

	public ShooterState getCurrentState() {
		return currentState;
	}

	public void setCurrentState(ShooterState currentState) {
		this.currentState = currentState;
	}

	public static Rotation2d hoodInterpolation(Supplier<Double> distanceFromTower) {
		return ShooterConstants.HOOD_INTERPOLATION_MAP.get(distanceFromTower.get());
	}

	public static Rotation2d flywheelInterpolation(Supplier<Double> distanceFromTower) {
		return ShooterConstants.FLYWHEEL_INTERPOLATION_MAP.get(distanceFromTower.get());
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

	private Rotation2d getLookAtTowerAngleForTurretRobotRelative(Translation2d target, Pose2d robotTranslation) {
		return FieldMath.getRelativeTranslation(robotTranslation,target).getAngle();
	}

	public Command lookAtTower(Translation2d target, Pose2d robotPose) {
		Rotation2d turretTargetRobotRelative = getLookAtTowerAngleForTurretRobotRelative(target, robotPose);
		if (
                Math.abs(turretTargetRobotRelative.getDegrees() - turret.getPosition().getDegrees()) < 360 - ShooterConstants.LENGTH_NOT_TO_TURN.getDegrees()
		) {
			return new ParallelCommandGroup(
				turret.getCommandsBuilder().setTargetPosition(Rotation2d.fromDegrees(turretTargetRobotRelative.getDegrees()%360)),
            new InstantCommand(() -> Logger.recordOutput(ShooterConstants.LOG_PATH + "/TurretTargetRobotRelative", Rotation2d.fromDegrees(turretTargetRobotRelative.getDegrees())))
			);
		}
        else {
            return new ParallelCommandGroup(
                    turret.getCommandsBuilder().stayInPlace(),
                    new InstantCommand(() -> Logger.recordOutput(ShooterConstants.LOG_PATH + "/TurretTargetRobotRelative", turretTargetRobotRelative))
            );
        }
	}

	public void Logger() {
		Logger.recordOutput(ShooterConstants.LOG_PATH + "/CurrentState", currentState);
	}

	private Command calibration() {
		return new ParallelCommandGroup(
			turret.getCommandsBuilder().setVoltage(ShooterConstants.turretCalibrationVoltage::get),
			hood.getCommandsBuilder().setVoltage(ShooterConstants.hoodCalibrationVoltage::get),
			flyWheel.getCommandBuilder().setVoltageAsSupplier(ShooterConstants.flywheelCalibrationVoltage::get)
		);
	}

}
