package frc.robot.subsystems.moduleAlon;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.joysticks.Axis;
import frc.joysticks.SmartJoystick;
import frc.utils.battery.BatteryUtil;
import org.littletonrobotics.junction.Logger;

import java.util.Set;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class ModuleCommandsBuilder {

	private final ModuleAlon moduleAlon;
	private SmartJoystick defaultJoystick;
	private final String logPath;

	public ModuleCommandsBuilder(ModuleAlon moduleAlon, String logPath) {
		this.logPath = logPath;
		this.moduleAlon = moduleAlon;
		moduleAlon.setDefaultCommand(new InstantCommand(() -> moduleAlon.stop(), moduleAlon));
		linearSetPower = (power) -> {
			moduleAlon.linearPower(power);
		};
		steerToPosition = (position) -> {
			moduleAlon.steerToPosition(position);
		};
	}

	public Command stopCommand() {
		return new RunCommand(() -> {
			moduleAlon.stop();
		}, moduleAlon).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
	}

	public void setDefaultJoystick(SmartJoystick defaultJoystick) {
		this.defaultJoystick = defaultJoystick;
	}

	private final Consumer<Double> linearSetPower;
	private final Consumer<Rotation2d> steerToPosition;


	public Command driveWithStick(Supplier<Double> xAxis, Supplier<Double> yAxis, boolean withLinearFlipping) {
		return new RunCommand(() -> {
			ModuleAlonMath.driveSwerve(steerToPosition, linearSetPower, xAxis, yAxis, withLinearFlipping, moduleAlon::getSteerPosition);
			Logger.recordOutput("axis/x", xAxis.get());
			Logger.recordOutput("axis/y", yAxis.get());
			Logger.recordOutput("axis/angle", Rotation2d.fromRadians(Math.atan2(yAxis.get(), xAxis.get())));
		}, moduleAlon).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
	}

	public Command driveWithLeftStick() {
		return driveWithLeftStick(defaultJoystick);
	}

	public Command driveWithRightStick() {
		return driveWithRightStick(defaultJoystick);
	}

	public Command driveWithLeftStick(SmartJoystick joystick) {
		return driveWithStick(() -> joystick.getAxisValue(Axis.LEFT_X), () -> joystick.getAxisValue(Axis.LEFT_Y), true);
	}

	public Command driveWithRightStick(SmartJoystick joystick) {
		return driveWithStick(() -> joystick.getAxisValue(Axis.RIGHT_X), () -> joystick.getAxisValue(Axis.RIGHT_Y), true);
	}

	public InstantCommand setNeutralMode(boolean brake) {
		return new InstantCommand(() -> {
			moduleAlon.setLinearNeutral(brake);
			moduleAlon.setSteerNeutral(brake);
		}, moduleAlon);
	}

	public RunCommand manualDriveCommand(Supplier<Double> power) {
		return new RunCommand(() -> {
			moduleAlon.linearPower(power.get());
			Logger.recordOutput(logPath + "/shouldPower", power.get() * BatteryUtil.DEFAULT_VOLTAGE);
		}, moduleAlon);
	}

	public void linearWithStickValue(SmartJoystick joystick, Axis axis) {
		moduleAlon.linearPower(joystick.getAxisValue(axis));
	}

	public static final double constantPower = 0.5;
	public static double steerToleranceDegrees = 10;

	public Command driveDistanceCommand(Rotation2d drive, Rotation2d angle) {
		FunctionalCommand steerToPosition = new FunctionalCommand(() -> moduleAlon.stop(), () -> {
			moduleAlon.steerToPosition(angle);
		}, (isInterrupted) -> {
			moduleAlon.stop();
		}, () -> MathUtil.isNear(angle.getDegrees(), moduleAlon.getSteerPosition().getDegrees(), steerToleranceDegrees), moduleAlon);

		Command driveToPosition = new DriveToPositionCommand(drive, moduleAlon);
		return new SequentialCommandGroup(steerToPosition, driveToPosition);
	}

	private static class DriveToPositionCommand extends Command {

		private Rotation2d originalPos;
		private final Rotation2d turns;
		private final ModuleAlon moduleAlon;

		public DriveToPositionCommand(Rotation2d turns, ModuleAlon moduleAlon) {
			this.turns = turns;
			this.moduleAlon = moduleAlon;
			this.addRequirements(moduleAlon);
		}

		@Override
		public void initialize() {
			originalPos = moduleAlon.getLinearPosition();
		}

		@Override
		public void execute() {
			moduleAlon.linearPower(constantPower * signOfTurns());
		}

		@Override
		public void end(boolean interrupted) {
			moduleAlon.stop();
		}

		@Override
		public boolean isFinished() {
			return moduleAlon.getLinearPosition().minus(turns).minus(originalPos).times(signOfTurns()).getRadians() > 0;
		}

		private int signOfTurns() {
			return (int) Math.signum(turns.getRadians());
		}

	}

	public RunCommand comboCommand() {
		return new RunCommand(() -> {
			moduleAlon.linearPower(1);
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

	public DeferredCommand realTimeChoice(Supplier<Boolean> shouldLeft) {
		return new DeferredCommand(() -> {
			return shouldLeft.get()
				? driveDistanceCommand(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(90))
				: driveDistanceCommand(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(-90));
		}, Set.of(moduleAlon));
	}

	public static final int loggityMouthCycles = 40;
	public static final int loggityRadius = 60;
	public static final double loggitySmileLowerMult = 1;
	public static final double loggitySmileUpperMult = 0.75;
	public static final int loggityEyeWidth = 10;
	public static final double loggityEyeMult = 1.25;
	public static final int loggityEyeDist = 17;
	public static final int loggityEyeRaise = 10;

	public static Command loggity() {
		return new Command() {

			static {
				Logger.recordOutput("Loggity/0", 0.0);
				Logger.recordOutput("Loggity/1", 0.0);
				Logger.recordOutput("Loggity/2", 0.0);
			}
			private int cycle;

			@Override
			public void initialize() {
				cycle = -loggityRadius;
			}

			@Override
			public void execute() {
				Logger.recordOutput("Loggity/0", logOutputMouth(cycle));
				double eye = Double.isNaN(logOutputEyes(cycle + loggityEyeDist))
					? logOutputEyes(cycle - loggityEyeDist)
					: logOutputEyes(cycle + loggityEyeDist);
				Logger.recordOutput("Loggity/1", eye);
				Logger.recordOutput("Loggity/2", logOutputFace(cycle));
				cycle++;
			}

			@Override
			public void end(boolean isInterrupted) {}

			@Override
			public boolean isFinished() {
				return cycle > loggityRadius;
			}

			private static double logOutputMouth(int cycle) {
				double op1 = -1 * Math.sqrt(loggityMouthCycles * loggityMouthCycles - cycle * cycle - Double.MIN_VALUE) * loggitySmileLowerMult;
				double op2 = -1 * Math.sqrt(loggityMouthCycles * loggityMouthCycles - cycle * cycle - Double.MIN_VALUE) * loggitySmileUpperMult;
				return cycle % 2 == 0 ? op1 : op2;
			}

			private static double logOutputEyes(int cycle) {
				return (cycle % 2 == 0 ? -1 : 1) * Math.sqrt(loggityEyeWidth * loggityEyeWidth - cycle * cycle) * loggityEyeMult
					+ loggityEyeRaise;
			}

			private static double logOutputFace(int cycle) {
				return (cycle % 2 == 0 ? -1 : 1) * Math.sqrt(loggityRadius * loggityRadius - cycle * cycle);
			}

		};
	}

}
