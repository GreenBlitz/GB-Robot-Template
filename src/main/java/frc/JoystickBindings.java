package frc;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.joysticks.BindSet;
import frc.robot.Robot;
import frc.joysticks.SmartJoystick;

public class JoystickBindings {

	public static void configureBindings(SmartJoystick joystick, Robot robot) {
		applySwerveBindings(joystick);
		applySecondBindings(joystick);
		applyNoBindingsBindings(joystick);
	}

	private static Trigger bindSetTrigger(SmartJoystick joystick, Trigger bind, BindSet bindSetRequirement) {
		return bind.and(bindSetTrigger(joystick, bindSetRequirement));
	}

	private static Trigger bindSetTrigger(SmartJoystick joystick, BindSet bindSetRequirement) {
		return new Trigger(() -> bindSetRequirement == joystick.getBindSet());
	}

	private static void applySwerveBindings(SmartJoystick joystick) {
		bindSetTrigger(joystick, joystick.A, BindSet.SWERVE).onTrue(new InstantCommand(() -> joystick.setBindSet(BindSet.cycleBindSet(joystick.getBindSet(), 1))));
		bindSetTrigger(joystick, joystick.B, BindSet.SWERVE).whileTrue(new RunCommand(() -> System.out.println("swerve")));
	}

	private static void applySecondBindings(SmartJoystick joystick) {
		bindSetTrigger(joystick, joystick.A, BindSet.SECOND).onTrue(new InstantCommand(() -> joystick.setBindSet(BindSet.cycleBindSet(joystick.getBindSet(), 1))));
		bindSetTrigger(joystick, joystick.B, BindSet.SECOND).whileTrue(new RunCommand(() -> System.out.println("second")));

	}

	private static void applyNoBindingsBindings(SmartJoystick joystick){
		bindSetTrigger(joystick, joystick.A, BindSet.NO_BINDINGS).onTrue(new InstantCommand(() -> joystick.setBindSet(BindSet.cycleBindSet(joystick.getBindSet(), 1))));
		bindSetTrigger(joystick, joystick.B, BindSet.NO_BINDINGS).whileTrue(new RunCommand(() -> System.out.println("empty")));

	}


}
