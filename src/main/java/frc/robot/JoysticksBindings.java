package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.Ports;
import frc.robot.subsystems.swerve.SwerveCommands;
import frc.utils.allianceutils.AlliancePose2d;
import frc.utils.joysticks.SmartJoystick;

public class JoysticksBindings {

	private static final SmartJoystick MAIN_JOYSTICK = new SmartJoystick(Ports.JoystickDriverStationPorts.MAIN);
	private static final SmartJoystick SECOND_JOYSTICK = new SmartJoystick(Ports.JoystickDriverStationPorts.SECOND);
	private static final SmartJoystick THIRD_JOYSTICK = new SmartJoystick(Ports.JoystickDriverStationPorts.THIRD);
	private static final SmartJoystick FOURTH_JOYSTICK = new SmartJoystick(Ports.JoystickDriverStationPorts.FOURTH);

	public static void configureBindings() {
		mainJoystickButtons();
		secondJoystickButtons();
		thirdJoystickButtons();
		fourthJoystickButtons();
	}

	private static void mainJoystickButtons() {
		SmartJoystick usedJoystick = MAIN_JOYSTICK;
		//bindings

		//TODO - return to last pose estimator
		usedJoystick.Y.onTrue(new InstantCommand(() -> RobotContainer.POSE_ESTIMATOR.setHeading(new Rotation2d())));
		usedJoystick.Y.onTrue(new InstantCommand(() ->
				RobotContainer.POSE_ESTIMATOR.resetPose(
						AlliancePose2d.fromBlueAlliancePose(5, 5, new Rotation2d())
				)));

		usedJoystick.A.onTrue(new InstantCommand(RobotContainer.SWERVE::resetByEncoder));

		usedJoystick.POV_DOWN.whileTrue(SwerveCommands.getDriveToPoseCommand(
				() -> AlliancePose2d.fromBlueAlliancePose(4, 4, new Rotation2d()),
				new PathConstraints(3.3, 3.3, 4, 4)
		));

		RobotContainer.SWERVE.setDefaultCommand(
				SwerveCommands.getOpenLoopFieldRelativeDriveCommand(
						() -> usedJoystick.getAxisValue(SmartJoystick.Axis.LEFT_Y),
						() -> usedJoystick.getAxisValue(SmartJoystick.Axis.LEFT_X),
						() -> usedJoystick.getAxisValue(SmartJoystick.Axis.RIGHT_X)
				)
		);
	}

	private static void secondJoystickButtons() {
		SmartJoystick usedJoystick = SECOND_JOYSTICK;
		//bindings
	}

	private static void thirdJoystickButtons() {
		SmartJoystick usedJoystick = THIRD_JOYSTICK;
		//bindings
	}

	private static void fourthJoystickButtons() {
		SmartJoystick usedJoystick = FOURTH_JOYSTICK;
		//bindings
	}


}
