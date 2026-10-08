package frc;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.joysticks.Axis;
import frc.joysticks.SmartJoystick;
import frc.utils.math.AngleMath;
import org.littletonrobotics.junction.Logger;

import java.util.Set;
import java.util.function.Supplier;

public class ModulesCommandBuilder {

	private ModuleAlon moduleAlon;
	private SmartJoystick defaultJoystick;

	public ModulesCommandBuilder(ModuleAlon moduleAlon) {
		this.moduleAlon = moduleAlon;
		moduleAlon.setDefaultCommand(new InstantCommand(() -> moduleAlon.stop(),moduleAlon).withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf));
	}

	public void setDefaultJoystick(SmartJoystick defaultJoystick) {
		this.defaultJoystick = defaultJoystick;
	}

	public static final double stickMinTolerance = .1;

	public RunCommand driveWithStick(Supplier<Double> xAxis, Supplier<Double> yAxis) {
		RunCommand command =new RunCommand(() -> {
			double x = xAxis.get();
			double y = yAxis.get();
			Logger.recordOutput("axis/x",x);
			Logger.recordOutput("axis/y",y);
			if (x*x+y*y>=stickMinTolerance*stickMinTolerance) {
				moduleAlon.steerToPosition(AngleMath.closestAngle(Math.atan2(y, x),moduleAlon.getSteerAngle().getRadians()));
				moduleAlon.linearSetPower(Math.sqrt(x * x + y * y));
			}
		});
		command.addRequirements(moduleAlon);
		return command;
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

	public InstantCommand setNeutralMode(boolean brake){
		return new InstantCommand(()->{
			NeutralModeValue neutralMode = brake?NeutralModeValue.Brake:NeutralModeValue.Coast;
			setNeutralModeToLinear(neutralMode);
			setNeutralModeToSteer(neutralMode);
		},moduleAlon);
	}

	public RunCommand manualDriveCommand(Supplier<Double> power){
		return new RunCommand(()->{moduleAlon.linearSetPower(power.get());},moduleAlon);
	}

	public void linearWithStickValue(SmartJoystick joystick, Axis axis) {
		moduleAlon.linearSetPower(joystick.getAxisValue(axis));
	}

	public void stopModule() {
		moduleAlon.stop();
	}

	private final double constantPower = 0.5;
	private final static double steerToleranceRadians = 0.1;

	public Command driveDistanceCommand(Rotation2d drive, Rotation2d angle) {
		FunctionalCommand steerToPosition = new FunctionalCommand(
			() -> moduleAlon.stop(),
			() -> moduleAlon.steerToPosition(angle.getRadians()),
			(b) -> moduleAlon.stop(),
			() -> MathUtil.isNear(angle.getRadians(), moduleAlon.getSteerAngle().getRadians(), steerToleranceRadians)
		);


		int signOfDrive = (int) Math.signum(drive.getRadians());
		Rotation2d[] originalPos = {null};
		FunctionalCommand driveToPosition = new FunctionalCommand(() -> {
			originalPos[0] = moduleAlon.getLinearAngle();
		},
			() -> moduleAlon.linearSetPower(constantPower),
			(b) -> moduleAlon.linearSetPower(0),
			() -> (originalPos[0].plus(drive).minus(moduleAlon.getLinearAngle()).times(signOfDrive).getRadians() <= 0)
		);
		//SequentialCommandGroup command = new SequentialCommandGroup(steerToPosition,driveToPosition);
		//command.addRequirements(moduleAlon);
		//command.withInterruptBehavior(Command.InterruptionBehavior.kCancelSelf);
		steerToPosition.addRequirements(moduleAlon);
		return steerToPosition;
		//NOTE THAT THIS IMPLEMENTATION CANNOT STAY, YOU NEED TO ADD ALL THE COMMENTED SHI
	}
	public RunCommand comboCommand(){
		return new RunCommand(()->{moduleAlon.linearSetPower(1);});
	}
	public Command printArmOpening(){
		return new InstantCommand(()->{
			System.out.println("opening arm");
		});
	}
	public Command fullyCircleDrive(){
		SequentialCommandGroup command =  new SequentialCommandGroup(
				driveDistanceCommand(Rotation2d.fromRotations(2),Rotation2d.fromDegrees(0)),
				driveDistanceCommand(Rotation2d.fromRotations(2),Rotation2d.fromDegrees(90)),
				driveDistanceCommand(Rotation2d.fromRotations(2),Rotation2d.fromDegrees(180)),
				new ParallelCommandGroup(
				printArmOpening(),
				driveDistanceCommand(Rotation2d.fromRotations(2),Rotation2d.fromDegrees(-90)))
		);
		command.addRequirements(moduleAlon);
		return command;
	}

	public DeferredCommand realTimeChoice(Supplier<Boolean> var){
		return new DeferredCommand(
				()->{return var.get()?
						driveDistanceCommand(Rotation2d.fromDegrees(0),Rotation2d.fromDegrees(90)):
						driveDistanceCommand(Rotation2d.fromDegrees(0),Rotation2d.fromDegrees(-90));
				}, Set.of(moduleAlon)
		);
	}

}
