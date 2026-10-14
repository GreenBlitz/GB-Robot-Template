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

	public Command setSolidGreenColor() {
		return led.asSubsystemCommand(new InstantCommand(() -> led.setPattern(LEDPattern.solid(Color.kLimeGreen))), "Set LED pattern");
	}

	public Command setSolidWhiteColor() {
		return led.asSubsystemCommand(new InstantCommand(() -> led.setPattern(LEDPattern.solid(Color.kWhite))), "Set LED pattern");
	}

	public Command setSolidRedColor() {
		return led.asSubsystemCommand(new InstantCommand(() -> led.setPattern(LEDPattern.solid(Color.kWhite))), "Set LED pattern");
	}

	public Command setSolidBlueColor() {
		return led.asSubsystemCommand(new InstantCommand(() -> led.setPattern(LEDPattern.solid(Color.kLimeGreen))), "Set LED pattern");
	}

	public Command setSolidYellowColor() {
		return led.asSubsystemCommand(new InstantCommand(() -> led.setPattern(LEDPattern.solid(Color.kWhite))), "Set LED pattern");
	}

	public Command setAlternatingGreenYellowColors() {
		return led.asSubsystemCommand(
			new InstantCommand(() -> led.setAlternatingColors(Color.kLimeGreen, Color.kYellow)),
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
