package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.math.util.Units;
import frc.robot.commands.calibration.FindP;
import frc.robot.constants.Ports;
import frc.robot.subsystems.swerve.SwerveCommands;
import frc.utils.devicewrappers.GBTalonFXPro;
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
		GBTalonFXPro motor = new GBTalonFXPro(0);
		motor.getConfigurator().apply(new Slot0Configs().withKP(1));
		usedJoystick.A.whileTrue(new FindP(
						motor,
						new PositionVoltage(0),
						0,
						0.5,
						3,
						Units.degreesToRotations(1),
						Units.degreesToRotations(0.1),
						Units.degreesToRotations(1),
						Units.degreesToRotations(50)
				)
		);
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
