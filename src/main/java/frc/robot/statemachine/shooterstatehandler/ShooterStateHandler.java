package frc.robot.statemachine.shooterstatehandler;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.statemachine.ShootingCalculations;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.VelocityPositionArm;
import frc.robot.subsystems.constants.flywheel.FlywheelConstants;
import frc.robot.subsystems.constants.hood.HoodConstants;
import frc.robot.subsystems.constants.turret.TurretConstants;
import frc.robot.subsystems.flywheel.FlyWheel;
import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class ShooterStateHandler {

	private final VelocityPositionArm turret;
	private final Arm hood;
	private final FlyWheel flyWheel;
	private final Supplier<ShootingParams> shootingParamsSupplier;
	private final String logPath;
	private boolean hasHoodBeenReset;
	private boolean hasTurretBeenReset;
	private ShooterState currentState;
	private double resetStartTime;

	public ShooterStateHandler(
		VelocityPositionArm turret,
		Arm hood,
		FlyWheel flyWheel,
		Supplier<ShootingParams> shootingParamsSupplier,
		String logPath
	) {
		this.turret = turret;
		this.hood = hood;
		this.flyWheel = flyWheel;
		this.shootingParamsSupplier = shootingParamsSupplier;
		this.currentState = ShooterState.STAY_IN_PLACE;
		this.hasHoodBeenReset = Robot.ROBOT_TYPE.isSimulation();
		this.hasTurretBeenReset = Robot.ROBOT_TYPE.isSimulation();
		this.logPath = logPath + "/ShooterStateHandler";
		this.resetStartTime = 0;
	}

	public ShooterState getCurrentState() {
		return currentState;
	}

	public Command setState(ShooterState shooterState) {
		Command command = switch (shooterState) {
			case STAY_IN_PLACE -> stayInPlace();
			case NEUTRAL -> neutral();
			case SHOOT -> shoot();
			case RESET_SUBSYSTEMS -> resetSubsystems();
			case CALIBRATION -> calibration();
		};
		return new ParallelCommandGroup(
			new InstantCommand(() -> Logger.recordOutput(logPath + "/CurrentState", shooterState.name())),
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

	private Command neutral() {
		return new ParallelCommandGroup(
			turret.asSubsystemCommand(
				new TurretSafeMoveToPosition(
					turret,
					() -> shootingParamsSupplier.get().targetTurretPosition(),
					() -> ShootingCalculations.getShootingParams().targetTurretVelocityRPS(),
					logPath
				),
				"Safe move to position"
			),
			hood.getCommandsBuilder().setTargetPosition(() -> shootingParamsSupplier.get().targetHoodPosition()),
			flyWheel.getCommandBuilder().setTargetVelocity(ShooterConstants.DEFAULT_FLYWHEEL_ROTATIONS_PER_SECOND)
		);
	}

	private Command shoot() {
		return new ParallelCommandGroup(
			turret.asSubsystemCommand(
				new TurretSafeMoveToPosition(
					turret,
					() -> shootingParamsSupplier.get().targetTurretPosition(),
					() -> shootingParamsSupplier.get().targetTurretVelocityRPS(),
					logPath
				),
				"Safe move to position"
			),
			hood.getCommandsBuilder().setTargetPosition(() -> shootingParamsSupplier.get().targetHoodPosition()),
			flyWheel.getCommandBuilder().setVelocityAsSupplier(() -> shootingParamsSupplier.get().targetFlywheelVelocityRPS())
		);
	}

	private Command resetSubsystems() {
		return new ParallelCommandGroup(new InstantCommand(() -> {
			resetStartTime = TimeUtil.getCurrentTimeSeconds();
		}),
			hood.getCommandsBuilder().setVoltageWithoutLimit(HoodConstants.RESET_HOOD_VOLTAGE, () -> hasHoodBeenReset()),
			turret.getCommandsBuilder().setVoltageWithoutLimit(TurretConstants.RESET_TURRET_VOLTAGE, () -> hasTurretBeenReset())
		);
	}

	private Command calibration() {
		return new ParallelCommandGroup(
			turret.getCommandsBuilder().setTargetPosition(() -> ShootingCalculations.getShootingParams().targetTurretPosition()),
			hood.getCommandsBuilder().setTargetPosition(() -> ShooterConstants.hoodCalibrationAngle.get()),
			flyWheel.getCommandBuilder().setVelocityAsSupplier(() -> ShooterConstants.flywheelCalibrationRotations.get())
		);
	}

	public void periodic() {
		if (HoodConstants.MINIMUM_POSITION.getRadians() > hood.getPosition().getRadians() && !hasHoodBeenReset) {
			hood.setPosition(HoodConstants.MINIMUM_POSITION);
		}
		if (TurretConstants.MAX_POSITION.getRadians() < turret.getPosition().getRadians() && !hasTurretBeenReset) {
			turret.setPosition(TurretConstants.MAX_POSITION);
		}

		if (!hasHoodBeenReset && hood.getCurrent() > HoodConstants.CURRENT_THRESHOLD_TO_RESET_POSITION) {
			hasHoodBeenReset = true;
		}
		if (
			TimeUtil.getCurrentTimeSeconds() - resetStartTime > TurretConstants.RESET_START_IGNORANCE_TIME_SECONDS
				&& !hasTurretBeenReset
				&& turret.getCurrent() > TurretConstants.CURRENT_THRESHOLD_TO_RESET_POSITION
		) {
			hasTurretBeenReset = true;
		}

		Logger.recordOutput(logPath + "/HasHoodBeenReset", hasHoodBeenReset);
		Logger.recordOutput(logPath + "/HasTurretBeenReset", hasTurretBeenReset);
		Logger.recordOutput(logPath + "/CurrentState", currentState);
	}

	public boolean hasABallBeenShot() {
		if (currentState == ShooterState.SHOOT || currentState == ShooterState.CALIBRATION) {
			return flyWheel.getAcceleration().getRotations() < FlywheelConstants.FLYWHEEL_ACCELERATION_THRESHOLD_FOR_SHOT.getRotations();
		}
		return false;
	}

	public boolean hasTurretBeenReset() {
		return hasTurretBeenReset;
	}

	public boolean hasHoodBeenReset() {
		return hasHoodBeenReset;
	}

	public boolean hasBeenFullyReset() {
		return hasTurretBeenReset && hasHoodBeenReset;
	}

}
