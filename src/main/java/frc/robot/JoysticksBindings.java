package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.calibration.FindKS;
import frc.robot.commands.calibration.FindP;
import frc.robot.constants.Phoenix6Constants;
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
		GBTalonFXPro motor = new GBTalonFXPro(0, Phoenix6Constants.CANIVORE_NAME);
		motor.getConfigurator().apply(new Slot0Configs().withKP(500));
		usedJoystick.A.whileTrue(new FindP(
						motor,
						new PositionVoltage(0),
						-0.2,//Rot
						0.2,//Rot
						5,//Sec
						Units.degreesToRotations(0.2),// tolerance
						Units.degreesToRotations(1),// factor
						Units.degreesToRotations(0),// minRange
						Units.degreesToRotations(50)// maxRange
				)
		);
		usedJoystick.B.whileTrue(new FindKS(0, motor, new VoltageOut(0)));
		
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
