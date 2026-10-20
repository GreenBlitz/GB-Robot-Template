package frc;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.joysticks.Axis;
import frc.joysticks.JoystickPorts;
import frc.joysticks.SmartJoystick;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.ChassisPowers;
import frc.robot.subsystems.tal.ModulePowers;

public class JoysticksBindings {

	private static final SmartJoystick MAIN_JOYSTICK = new SmartJoystick(JoystickPorts.MAIN, true);
	private static final SmartJoystick SECOND_JOYSTICK = new SmartJoystick(JoystickPorts.SECOND);
	private static final SmartJoystick THIRD_JOYSTICK = new SmartJoystick(JoystickPorts.THIRD);
	private static final SmartJoystick FOURTH_JOYSTICK = new SmartJoystick(JoystickPorts.FOURTH);
	private static final SmartJoystick FIFTH_JOYSTICK = new SmartJoystick(JoystickPorts.FIFTH);
	private static final SmartJoystick SIXTH_JOYSTICK = new SmartJoystick(JoystickPorts.SIXTH);

	private static final ChassisPowers chassisDriverInputs = new ChassisPowers();
	private static final ModulePowers moduleInputs = new ModulePowers();


	public static void configureBindings(Robot robot) {
		robot.getSwerve().setDriversPowerInputs(chassisDriverInputs);
		robot.getModuleTal().setModulePowers(moduleInputs);

		mainJoystickButtons(robot);
		secondJoystickButtons(robot);
		thirdJoystickButtons(robot);
		fourthJoystickButtons(robot);
		fifthJoystickButtons(robot);
		sixthJoystickButtons(robot);
	}

	public static void updateChassisDriverInputs() {
		if (MAIN_JOYSTICK.isConnected()) {
			chassisDriverInputs.xPower = MAIN_JOYSTICK.getAxisValue(Axis.LEFT_Y);
			chassisDriverInputs.yPower = MAIN_JOYSTICK.getAxisValue(Axis.LEFT_X);
			chassisDriverInputs.rotationalPower = MAIN_JOYSTICK.getAxisValue(Axis.RIGHT_X);
		} else if (THIRD_JOYSTICK.isConnected()) {
			chassisDriverInputs.xPower = THIRD_JOYSTICK.getAxisValue(Axis.LEFT_Y);
			chassisDriverInputs.yPower = THIRD_JOYSTICK.getAxisValue(Axis.LEFT_X);
			chassisDriverInputs.rotationalPower = THIRD_JOYSTICK.getAxisValue(Axis.RIGHT_X);
		} else {
			chassisDriverInputs.xPower = 0;
			chassisDriverInputs.yPower = 0;
			chassisDriverInputs.rotationalPower = 0;
		}
	}

	public static void updateModuleTal() {
		if (MAIN_JOYSTICK.isConnected()) {
			moduleInputs.xPower = MAIN_JOYSTICK.getAxisValue(Axis.LEFT_X);
			moduleInputs.yPower = MAIN_JOYSTICK.getAxisValue(Axis.LEFT_Y);
		}
	}


	private static void mainJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = MAIN_JOYSTICK;
		// bindings...
		usedJoystick.A.whileTrue(new InstantCommand(robot.getMotor()::MoveForwardHalfPower));
		usedJoystick.B.whileTrue(new InstantCommand(robot.getMotor()::stopMotor));
		usedJoystick.Y.onTrue(new InstantCommand(() -> robot.getMotor().pidWithPositionVoltage(100.0)));
		usedJoystick.POV_DOWN.onTrue(new InstantCommand(robot.getMotor()::setPoisition));
		usedJoystick.POV_RIGHT.onTrue(robot.getModuleTal().getCommandBuilder().FullyCircleDrive());
		usedJoystick.POV_LEFT.onTrue(new RunCommand(() -> robot.getModuleTal().steerToAngle(90)));
		usedJoystick.X.onTrue(
			robot.getModuleTal()
				.getCommandBuilder()
				.driveModuleByJoystick(
					() -> robot.getModuleTal().getTargetJoystickAngle().get(),
					() -> robot.getModuleTal().getTargetMotorPower().get()
				)
		);
		Trigger combo = usedJoystick.R1.and(usedJoystick.L1);
		combo.onTrue(robot.getModuleTal().getCommandBuilder().setDrivePower(() -> 0.5));
	}

	private static void secondJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = SECOND_JOYSTICK;
		// bindings...
	}

	private static void thirdJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = THIRD_JOYSTICK;
		// bindings...
	}

	private static void fourthJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = FOURTH_JOYSTICK;
		// bindings...
	}

	private static void fifthJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = FIFTH_JOYSTICK;
		// bindings...
	}

	private static void sixthJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = SIXTH_JOYSTICK;
		// bindings...
	}

}
