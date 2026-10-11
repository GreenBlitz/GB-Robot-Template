package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import static edu.wpi.first.units.Units.MetersPerSecond;

public class LEDCommandsBuilder {

	private final LED led;

	protected LEDCommandsBuilder(LED led) {
		this.led = led;
	}

	public Command setSolidColor(Color color) {
		return led.asSubsystemCommand(new InstantCommand(() -> led.setPattern(LEDPattern.solid(color))), "Set LED pattern");
	}

	public Command setAlternatingGreenWhiteColors(LEDPattern pattern) {
		return led.asSubsystemCommand(
			new InstantCommand(() -> led.setAlternatingColors(Color.kLimeGreen, Color.kWhite)),
			"Set Alternating Led Pattern"
		);
	}

	public Command setRainbowGradient() {
		return led.asSubsystemCommand(
			new InstantCommand(() -> led.setPattern(LEDPattern.rainbow(LED.ledConstants.saturation(), LED.ledConstants.value()))),
			"Set Rainbow LED Pattern"
		);
	}

	public Command blink() {
		return led.asSubsystemCommand(new RunCommand(led::applyPattern).beforeStarting(led::blinkWithCurrentPattern), "Set LED to Blink");
	}

	public Command enableScroll() {
		return led.asSubsystemCommand(
			new InstantCommand(() -> led.enableScroll(MetersPerSecond.of(LED.ledConstants.scrollSpeedMetersPerSecond()))),
			"Set LED to Scroll"
		);
	}

}
