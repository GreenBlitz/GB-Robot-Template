package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import static edu.wpi.first.units.Units.MetersPerSecond;

public class LEDCommandsBuilder {

	private final LED led;

	private Command blinkCommand;

	protected LEDCommandsBuilder(LED led) {
		this.led = led;
	}

	public Command setSolidColor(Color color) {
		return led.asSubsystemCommand(new InstantCommand(() -> led.setPattern(LEDPattern.solid(Color.kWhite))), "Set LED pattern");
	}

	public Command setTwoAlternatingColors(Color a, Color b) {
		return led.asSubsystemCommand(new InstantCommand(() -> led.setAlternatingColors(a, b)), "Set Alternating Led Pattern");
	}

	public Command setRainbowGradient() {
		return led.asSubsystemCommand(
			new InstantCommand(() -> led.setPattern(LEDPattern.rainbow(LED.ledConstants.SATURATION(), LED.ledConstants.VALUE()))),
			"Set Rainbow LED Pattern"
		);
	}

	public Command enableBlink() {
		blinkCommand = led
			.asSubsystemCommand(new RunCommand(led::applyPattern).beforeStarting(led::blinkWithCurrentPattern), "Enable LED enableBlink");
		return blinkCommand;
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
			new InstantCommand(() -> led.enableScroll(MetersPerSecond.of(LED.ledConstants.SCROLL_SPEED_PER_METER_PER_SECOND()))),
			"Enable LED Scroll"
		);
	}

	public Command disableScroll() {
		return led.asSubsystemCommand(new InstantCommand(() -> led.setPattern(led.getBasePattern())), "Disable LED Scroll");
	}

}
