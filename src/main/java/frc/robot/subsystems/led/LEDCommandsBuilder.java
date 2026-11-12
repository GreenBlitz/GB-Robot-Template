package frc.robot.subsystems.led;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class LEDCommandsBuilder {

	private final LED led;

	private Command blinkCommand;

	protected LEDCommandsBuilder(LED led) {
		this.led = led;
	}

	public Command setGenericPattern(LEDPattern pattern) {
		return led.asSubsystemCommand(new InstantCommand(() -> led.setGenericPattern(pattern)), "Set generic LED pattern");
	}

	public Command setSolidColor(Color color) {
		return led.asSubsystemCommand(new InstantCommand(() -> led.setGenericPattern(LEDPattern.solid(Color.kWhite))), "Set solid LED color");
	}

	public Command setTwoAlternatingColors(Color a, Color b) {
		return led.asSubsystemCommand(new InstantCommand(() -> led.setAlternatingColors(a, b)), "Set Alternating Led Pattern");
	}

	public Command setRainbowGradient() {
		return led.asSubsystemCommand(
			new InstantCommand(() -> led.setGenericPattern(LEDPattern.rainbow(LED.ledConstants.SATURATION(), LED.ledConstants.VALUE()))),
			"Set Rainbow LED Pattern"
		);
	}

	public Command enableBlink(double timeBetweenBlinksSeconds) {
		return led.asSubsystemCommand(
			new RunCommand(led::blinkWithCurrentPattern).beforeStarting(() -> led.setTimeBetweenBlinksSeconds(timeBetweenBlinksSeconds)),
			"Enable LED enableBlink"
		);
	}

	public Command disableBlink() {
		return led.asSubsystemCommand(new InstantCommand(() -> {
			if (blinkCommand != null) {
				blinkCommand.cancel();
			}
		}), "Disable LED enableBlink");
	}

	public Command enableScroll() {
		return led.asSubsystemCommand(
				new InstantCommand(() -> led.enableScroll(LED.ledConstants.SCROLL_SPEED_PER_METER_PER_SECOND())),
				"Enable LED Scroll"
		);
	}

	public Command enableScroll(LinearVelocity metersPerSecondVelocity) {
		return led.asSubsystemCommand(
				new InstantCommand(() -> led.enableScroll(metersPerSecondVelocity)),
				"Enable LED Scroll"
		);
	}

	public Command disableScroll() {
		return led.asSubsystemCommand(new InstantCommand(() -> led.setGenericPattern(led.getBasePattern())), "Disable LED Scroll");
	}

}
