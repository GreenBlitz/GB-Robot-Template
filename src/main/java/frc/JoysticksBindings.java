package frc;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.joysticks.Axis;
import frc.joysticks.JoystickPorts;
import frc.joysticks.SmartJoystick;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.ChassisPowers;
import frc.robot.subsystems.swerve.Swerve;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class JoysticksBindings {

	private static final SmartJoystick MAIN_JOYSTICK = new SmartJoystick(JoystickPorts.MAIN);
	private static final SmartJoystick SECOND_JOYSTICK = new SmartJoystick(JoystickPorts.SECOND);
	private static final SmartJoystick THIRD_JOYSTICK = new SmartJoystick(JoystickPorts.THIRD);
	private static final SmartJoystick FOURTH_JOYSTICK = new SmartJoystick(JoystickPorts.FOURTH);
	private static final SmartJoystick FIFTH_JOYSTICK = new SmartJoystick(JoystickPorts.FIFTH);
	private static final SmartJoystick SIXTH_JOYSTICK = new SmartJoystick(JoystickPorts.SIXTH);

	public static void configureBindings(Robot robot) {
		mainJoystickButtons(robot);
		secondJoystickButtons(robot);
		thirdJoystickButtons(robot);
		fourthJoystickButtons(robot);
		fifthJoystickButtons(robot);
		sixthJoystickButtons(robot);
	}

	public static void setDriversInputsToSwerve(Swerve swerve) {
		if (MAIN_JOYSTICK.isConnected()) {
			swerve.setDriversPowerInputs(
				new ChassisPowers(
					MAIN_JOYSTICK.getAxisValue(Axis.LEFT_Y),
					MAIN_JOYSTICK.getAxisValue(Axis.LEFT_X),
					MAIN_JOYSTICK.getAxisValue(Axis.RIGHT_X)
				)
			);
		} else if (THIRD_JOYSTICK.isConnected()) {
			swerve.setDriversPowerInputs(
				new ChassisPowers(
					THIRD_JOYSTICK.getAxisValue(Axis.LEFT_Y),
					THIRD_JOYSTICK.getAxisValue(Axis.LEFT_X),
					THIRD_JOYSTICK.getAxisValue(Axis.RIGHT_X)
				)
			);
		} else {
			swerve.setDriversPowerInputs(new ChassisPowers(0, 0, 0));
		}
	}

	private static void mainJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = MAIN_JOYSTICK;
		// bindings...
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
		Rotation2d armTargetPosition = robot.getArm().getPosition();
		Logger.recordOutput(robot.getArm().getLogPath() + "/ Arm target position", (DoubleSupplier) armTargetPosition::getRotations);

		usedJoystick.A.onTrue(robot.getArm().getCommandsBuilder().moveToPosition(Rotation2d.fromDegrees(-40)));
		usedJoystick.B.onTrue(robot.getArm().getCommandsBuilder().moveToPosition(Rotation2d.fromDegrees(0)));
		usedJoystick.X.onTrue(robot.getArm().getCommandsBuilder().moveToPosition(Rotation2d.fromDegrees(90)));
		usedJoystick.Y.onTrue(robot.getArm().getCommandsBuilder().moveToPosition(Rotation2d.fromDegrees(200)));

		usedJoystick.POV_DOWN.onTrue(robot.getArm().getSysIdCalibrator().getSysIdCommand(true, SysIdRoutine.Direction.kForward));
		usedJoystick.POV_UP.onTrue(robot.getArm().getSysIdCalibrator().getSysIdCommand(true, SysIdRoutine.Direction.kReverse));
		usedJoystick.POV_DOWN.onTrue(robot.getArm().getSysIdCalibrator().getSysIdCommand(false, SysIdRoutine.Direction.kForward));
		usedJoystick.POV_DOWN.onTrue(robot.getArm().getSysIdCalibrator().getSysIdCommand(false, SysIdRoutine.Direction.kReverse));
	}

	private static void sixthJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = SIXTH_JOYSTICK;
		// bindings...
	}

}
