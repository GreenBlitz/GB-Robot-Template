package frc.robot.subsystems.moduleAlon;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.constants.AlonModuleConstants;
import frc.joysticks.Axis;
import frc.joysticks.SmartJoystick;
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
			new InstantCommand(() -> moduleAlon.stop(), moduleAlon)
		);
	}

	public RunCommand stopCommand(){
		return new RunCommand(()->{moduleAlon.stop();},moduleAlon);
	}

	public void setDefaultJoystick(SmartJoystick defaultJoystick) {
		this.defaultJoystick = defaultJoystick;
	}

	public Command driveWithStick(Supplier<Double> xAxis, Supplier<Double> yAxis) {
		return new RunCommand(() -> {
			double x = xAxis.get();
			double y = yAxis.get();
			Logger.recordOutput(logPath+"/axis/x", x);
			Logger.recordOutput(logPath+"/axis/y", y);
			Logger.recordOutput(logPath+"/axis/angle",Rotation2d.fromRadians(Math.atan2(y, x)));
			Logger.recordOutput(logPath+"/axis/angleInRange",AngleMath.findAngleInRange(Rotation2d.fromRadians(Math.atan2(y, x)), moduleAlon.getSteerPosition()));
			if (x * x + y * y >= AlonModuleConstants.moduleAlonStickTolerance * AlonModuleConstants.moduleAlonStickTolerance) {
				moduleAlon.steerToPosition(AngleMath.findAngleInRange(Rotation2d.fromRadians(Math.atan2(y, x)), moduleAlon.getSteerPosition()));
				moduleAlon.linearPower(Math.sqrt(x * x + y * y));
			} else {
				moduleAlon.stop();
			}
		}, moduleAlon).withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming);
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


	public InstantCommand setNeutralMode(boolean brake) {
		return new InstantCommand(() -> {
			moduleAlon.setLinearNeutral(brake);
			moduleAlon.setSteerNeutral(brake);
		}, moduleAlon);
	}

	public RunCommand manualDriveCommand(Supplier<Double> power) {
		return new RunCommand(()->moduleAlon.linearPower(power.get()), moduleAlon);
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
		}, () -> MathUtil.isNear(angle.getDegrees(), moduleAlon.getSteerPosition().getDegrees(), steerToleranceDegrees),
				moduleAlon);

		Command driveToPosition = new DriveToPositionCommand(drive, moduleAlon);
		return new SequentialCommandGroup(
			steerToPosition,
			driveToPosition
		);
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

}
