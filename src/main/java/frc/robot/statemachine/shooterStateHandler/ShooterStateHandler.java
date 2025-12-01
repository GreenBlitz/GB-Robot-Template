package frc.robot.statemachine.shooterstatehandler;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.flywheel.FlyWheel;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class ShooterStateHandler {

	private final Arm turret;
	private final Arm hood;
	private final FlyWheel flyWheel;
	private final Supplier<Double> distanceFromTower;
	public ShooterState currentState;

	public ShooterStateHandler(Arm turret, Arm hood, FlyWheel flyWheel, Supplier<Double> distanceFromTower) {
		this.turret = turret;
		this.hood = hood;
		this.flyWheel = flyWheel;
		this.distanceFromTower = distanceFromTower;
		this.currentState = ShooterState
				.STAY_IN_PLACE;
	}

	public static Supplier<Rotation2d> hoodInterpolation(Supplier<Double> distanceFromTower) {
		return () -> ShooterConstants.HOOD_INTERPOLATION_MAP.get(distanceFromTower.get());
	}

	public static Supplier<Rotation2d> flywheelInterpolation(Supplier<Double> distanceFromTower) {
		return () -> ShooterConstants.FLYWHEEL_INTERPOLATION_MAP.get(distanceFromTower.get());
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
			flyWheel.getCommandBuilder().setVelocityAsSupplier(flywheelInterpolation(distanceFromTower))
		);
	}

	private Command calibration() {
		return new ParallelCommandGroup(
			turret.getCommandsBuilder().setTargetPosition(ShooterConstants.turretCalibrationAngle::get),
			hood.getCommandsBuilder().setTargetPosition(ShooterConstants.hoodCalibrationAngle::get),
			flyWheel.getCommandBuilder().setVelocityAsSupplier(ShooterConstants.flywheelCalibrationRotations::get)
		);
	}

}
