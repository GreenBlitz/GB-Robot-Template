package frc;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.joysticks.Axis;
import frc.joysticks.JoystickPorts;
import frc.joysticks.SmartJoystick;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.ChassisPowers;
import frc.utils.LimelightHelpers;
import frc.utils.calibration.limelightcalibration.CameraCalibration;
import frc.utils.calibration.limelightcalibration.LimelightCalculations;
import org.littletonrobotics.junction.Logger;

public class JoysticksBindings {

	private static final SmartJoystick MAIN_JOYSTICK = new SmartJoystick(JoystickPorts.MAIN);
	private static final SmartJoystick SECOND_JOYSTICK = new SmartJoystick(JoystickPorts.SECOND);
	private static final SmartJoystick THIRD_JOYSTICK = new SmartJoystick(JoystickPorts.THIRD);
	private static final SmartJoystick FOURTH_JOYSTICK = new SmartJoystick(JoystickPorts.FOURTH);
	private static final SmartJoystick FIFTH_JOYSTICK = new SmartJoystick(JoystickPorts.FIFTH);
	private static final SmartJoystick SIXTH_JOYSTICK = new SmartJoystick(JoystickPorts.SIXTH);

	private static final ChassisPowers chassisDriverInputs = new ChassisPowers();

	public static void configureBindings(Robot robot) {
		robot.getSwerve().setDriversPowerInputs(chassisDriverInputs);

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

	private static void mainJoystickButtons(Robot robot) {
		SmartJoystick usedJoystick = MAIN_JOYSTICK;
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
		Pose3d tagToRobot = new Pose3d(0.46, 0, 0.155, new Rotation3d(0, 0, 180));
//		usedJoystick.A.onTrue(new CameraCalibration(robot.getLimelightFour(), tagToRobot));
//		usedJoystick.B.onTrue(new CameraCalibration(robot.getLimelightThreeGB(), tagToRobot));
		usedJoystick.X.onTrue(new RunCommand(() -> LimelightCalculations.getCameraToRobot(robot.getLimelightFour().getRobotRelativeCameraPose(), tagToRobot)));
		usedJoystick.Y.onTrue(new RunCommand(() -> {
			Pose3d cameraToRobot = LimelightCalculations.getCameraToRobot(LimelightHelpers.getTargetPose3d_CameraSpace(robot.getLimelightThreeGB().getName()), tagToRobot);
			Logger.recordOutput("AAAAAAAAAAAAAAA", cameraToRobot);
			Logger.recordOutput("AAAAAAAAAAAAAAA/roll", Rotation2d.fromRadians(cameraToRobot.getRotation().getX()).getDegrees());
			Logger.recordOutput("AAAAAAAAAAAAAAA/pitch", Rotation2d.fromRadians(cameraToRobot.getRotation().getY()).getDegrees());
			Logger.recordOutput("AAAAAAAAAAAAAAA/yaw", Rotation2d.fromRadians(cameraToRobot.getRotation().getZ()).getDegrees());
		}));


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
