package frc;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.joysticks.Axis;
import frc.joysticks.SmartJoystick;

import java.util.function.Supplier;

public class ModulesCommandBuilder {

	private ModuleAlon moduleAlon;
	private SmartJoystick defaultJoystick;

	public ModulesCommandBuilder(ModuleAlon moduleAlon) {
		this.moduleAlon = moduleAlon;
	}

	public void setDefaultJoystick(SmartJoystick defaultJoystick) {
		this.defaultJoystick = defaultJoystick;
	}

	public RunCommand driveWithStick(Supplier<Double> xAxis, Supplier<Double> yAxis) {
		return new RunCommand(() -> {
			double x = xAxis.get();
			double y = yAxis.get();
			moduleAlon.steerToPosition(Math.atan2(y, x));
			moduleAlon.linearSetPower(Math.sqrt(x * x + y * y));
		});
	}

	public RunCommand driveWithLeftStick() {
		return driveWithLeftStick(defaultJoystick);
	}

	public RunCommand driveWithRightStick() {
		return driveWithRightStick(defaultJoystick);
	}

	public RunCommand driveWithLeftStick(SmartJoystick joystick) {
		return driveWithStick(() -> joystick.getAxisValue(Axis.LEFT_X), () -> joystick.getAxisValue(Axis.LEFT_Y));
	}

	public RunCommand driveWithRightStick(SmartJoystick joystick) {
		return driveWithStick(() -> joystick.getAxisValue(Axis.RIGHT_X), () -> joystick.getAxisValue(Axis.RIGHT_Y));
	}

	public void logAll() {
		moduleAlon.logAll();
	}

	public void setNeutralModeToLinear(NeutralModeValue mode) {
		moduleAlon.setLinearNeutral(mode);
	}

	public void setNeutralModeToSteer(NeutralModeValue mode) {
		moduleAlon.setSteerNeutral(mode);
	}

	public void linearWithStickValue(SmartJoystick joystick, Axis axis) {
		moduleAlon.linearSetPower(joystick.getAxisValue(axis));
	}

	public void stopModule() {
		moduleAlon.stop();
	}

	private final double constantPower = .5;
	private final static double steerToleranceRadians = .01;

	public FunctionalCommand driveDistanceCommand(Rotation2d drive, Rotation2d angle) {
		FunctionalCommand command = new FunctionalCommand(
			() -> moduleAlon.stop(),
			() -> moduleAlon.steerToPosition(angle.getRadians()),
			(b) -> moduleAlon.stop(),
			() -> MathUtil.isNear(angle.getRadians(), moduleAlon.getSteerAngle().getRadians(), steerToleranceRadians)
		);
		command.addRequirements(moduleAlon);
		int signOfDrive = (int) Math.signum(drive.getRadians());
		Rotation2d[] originalPos = {null};
		FunctionalCommand driveToPosition = new FunctionalCommand(() -> {
			originalPos[0] = moduleAlon.getLinearAngle();
		},
			() -> moduleAlon.linearSetPower(constantPower),
			(b) -> moduleAlon.linearSetPower(0),
			() -> (originalPos[0].plus(drive).minus(moduleAlon.getLinearAngle()).times(signOfDrive).getRadians() <= 0)
		);
		driveToPosition.withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf);
		command.andThen(driveToPosition);
		command.withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf);
		return command;
	}
	public InstantCommand comboCommand(){
		return new InstantCommand(()->{moduleAlon.linearSetPower(0.5);});
	}
	public Command printArmOpening(){
		return new InstantCommand(()->{
			System.out.println("opening arm");
		});
	}
	public Command fullyCircleDrive(){
		return new SequentialCommandGroup(
				driveDistanceCommand(Rotation2d.fromRotations(2),Rotation2d.fromDegrees(0)),
				driveDistanceCommand(Rotation2d.fromRotations(2),Rotation2d.fromDegrees(90)),
				driveDistanceCommand(Rotation2d.fromRotations(2),Rotation2d.fromDegrees(180)),
				new ParallelCommandGroup(
				printArmOpening(),
				driveDistanceCommand(Rotation2d.fromRotations(2),Rotation2d.fromDegrees(-90)))
		);
	}

}
