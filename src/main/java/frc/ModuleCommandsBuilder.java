package frc;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.joysticks.Axis;
import frc.joysticks.SmartJoystick;
import frc.robot.RobotConstants;
import frc.utils.math.AngleMath;
import org.littletonrobotics.junction.Logger;

import java.util.Set;
import java.util.function.Supplier;

public class ModuleCommandsBuilder {

	private final ModuleAlon moduleAlon;
	private SmartJoystick defaultJoystick;
	private final String logPath;

	public ModuleCommandsBuilder(ModuleAlon moduleAlon, String logPath) {
		this.logPath = logPath;
		this.moduleAlon = moduleAlon;
		moduleAlon.setDefaultCommand(
			new InstantCommand(() -> moduleAlon.stop(), moduleAlon).withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)
		);
	}

	public void setDefaultJoystick(SmartJoystick defaultJoystick) {
		this.defaultJoystick = defaultJoystick;
	}

	public Command driveWithStick(Supplier<Double> xAxis, Supplier<Double> yAxis) {
		RunCommand command = new RunCommand(() -> {
			double x = xAxis.get();
			double y = yAxis.get();
			Logger.recordOutput("axis/x", x);
			Logger.recordOutput("axis/y", y);
			if (x * x + y * y >= RobotConstants.moduleAlonStickTolerance * RobotConstants.moduleAlonStickTolerance) {
				moduleAlon.steerToPosition(AngleMath.findAngleInRange(Rotation2d.fromRadians(Math.atan2(y, x)), moduleAlon.getSteerAngle()));
				moduleAlon.linearSetPower(Math.sqrt(x * x + y * y));
			}
		});
		command.addRequirements(moduleAlon);
		return command.withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
	}

	public Command driveWithLeftStick() {
		return driveWithLeftStick(defaultJoystick);
	}

	public Command driveWithRightStick() {
		return driveWithRightStick(defaultJoystick);
	}

	public Command driveWithLeftStick(SmartJoystick joystick) {
		return driveWithStick(() -> joystick.getAxisValue(Axis.LEFT_X), () -> joystick.getAxisValue(Axis.LEFT_Y));
	}

	public Command driveWithRightStick(SmartJoystick joystick) {
		return driveWithStick(() -> joystick.getAxisValue(Axis.RIGHT_X), () -> joystick.getAxisValue(Axis.RIGHT_Y));
	}

	public void setNeutralModeToLinear(NeutralModeValue mode) {
		moduleAlon.setLinearNeutral(mode);
	}

	public void setNeutralModeToSteer(NeutralModeValue mode) {
		moduleAlon.setSteerNeutral(mode);
	}

	public InstantCommand setNeutralMode(boolean brake) {
		return new InstantCommand(() -> {
			NeutralModeValue neutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
			setNeutralModeToLinear(neutralMode);
			setNeutralModeToSteer(neutralMode);
		}, moduleAlon);
	}

	public RunCommand manualDriveCommand(Supplier<Double> power) {
		return new RunCommand(() -> {
			moduleAlon.linearSetPower(power.get());
		}, moduleAlon);
	}

	public void linearWithStickValue(SmartJoystick joystick, Axis axis) {
		moduleAlon.linearSetPower(joystick.getAxisValue(axis));
	}

	public static final double constantPower = 0.5;
	public static double steerToleranceDegrees = 10;

	public static Command increaseTolerance() {
		return new InstantCommand(() -> {
			steerToleranceDegrees += 5;
			System.out.println("steer tolerance:" + steerToleranceDegrees);
		});
	}

	public static Command decreaseTolerance() {
		return new InstantCommand(() -> {
			steerToleranceDegrees -= 5;
			System.out.println("steer tolerance:" + steerToleranceDegrees);
		});
	}

	public Command driveDistanceCommand(Rotation2d drive, Rotation2d angle) {
		FunctionalCommand steerToPosition = new FunctionalCommand(() -> moduleAlon.stop(), () -> {
			moduleAlon.steerToPosition(angle);
			Logger.recordOutput(logPath + "/targetAngle", angle);
		}, (b) -> {
			moduleAlon.stop();
		}, () -> {
			return MathUtil.isNear(angle.getDegrees(), moduleAlon.getSteerAngle().getDegrees(), steerToleranceDegrees);
		});


		Command driveToPosition = new DriveToPositionCommand(drive, moduleAlon);
		steerToPosition.addRequirements(moduleAlon);
		driveToPosition.addRequirements(moduleAlon);
		// command.steerToPosition(moduleAlon);
		// command.withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf);
		// steerToPosition.addRequirements(moduleAlon);
		return new SequentialCommandGroup(
			steerToPosition.withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf),
			driveToPosition.withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf)
		);
		// NOTE THAT THIS IMPLEMENTATION CANNOT STAY, YOU NEED TO ADD ALL THE COMMENTED SHI
	}

	private static class DriveToPositionCommand extends Command {

		private Rotation2d originalPos;
		private final Rotation2d turns;
		private final ModuleAlon moduleAlon;

		public DriveToPositionCommand(Rotation2d turns, ModuleAlon moduleAlon) {
			this.turns = turns;
			this.moduleAlon = moduleAlon;
		}

		@Override
		public void initialize() {
			originalPos = moduleAlon.getLinearAngle();
		}

		@Override
		public void execute() {
			moduleAlon.linearSetPower(constantPower * signOfTurns());
		}

		@Override
		public void end(boolean interrupted) {
			moduleAlon.stop();
		}

		@Override
		public boolean isFinished() {
			return moduleAlon.getLinearAngle().minus(turns).minus(originalPos).times(signOfTurns()).getRadians() > 0;
		}

		private int signOfTurns() {
			return (int) Math.signum(turns.getRadians());
		}

	}

	public RunCommand comboCommand() {
		return new RunCommand(() -> {
			moduleAlon.linearSetPower(1);
		});
	}

	public Command printArmOpening() {
		return new InstantCommand(() -> {
			System.out.println("opening arm");
		});
	}

	public Command fullyCircleDrive() {
		SequentialCommandGroup command = new SequentialCommandGroup(
			driveDistanceCommand(Rotation2d.fromRotations(2), Rotation2d.fromDegrees(0)),
			driveDistanceCommand(Rotation2d.fromRotations(2), Rotation2d.fromDegrees(90)),
			driveDistanceCommand(Rotation2d.fromRotations(2), Rotation2d.fromDegrees(180)),
			new ParallelCommandGroup(printArmOpening(), driveDistanceCommand(Rotation2d.fromRotations(2), Rotation2d.fromDegrees(-90)))
		);
		command.addRequirements(moduleAlon);
		return command;
	}

	public DeferredCommand realTimeChoice(Supplier<Boolean> var) {
		return new DeferredCommand(() -> {
			return var.get()
				? driveDistanceCommand(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(90))
				: driveDistanceCommand(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(-90));
		}, Set.of(moduleAlon));
	}

}
