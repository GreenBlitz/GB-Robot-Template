package frc.robot.statemachine.shooterStateHandler;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.flywheel.FlyWheel;
import frc.utils.InterpolationMap;

import java.util.Map;
import java.util.function.Supplier;

public class ShooterStateHandler {

	private final Arm turret;
	private final Arm hood;
	private final FlyWheel flyWheel;
	private ShooterState currentState;
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

	public ShooterStateHandler(Arm turret, Arm hood, FlyWheel flyWheel, Supplier<Double> distanceFromTower) {
		this.turret = turret;
		this.hood = hood;
		this.flyWheel = flyWheel;
		this.distanceFromTower = distanceFromTower;
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
			case SHOOT_WHILE_DRIVE -> shootWhileDrive();
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

	private Command shootWhileDrive() {
		return new ParallelCommandGroup(
			turret.getCommandsBuilder().stayInPlace(),
			hood.getCommandsBuilder().setTargetPosition(hoodInterpolation(distanceFromTower)),
			flyWheel.getCommandBuilder().setTargetVelocity(flywheelInterpolation(distanceFromTower))
		);
	}

}
