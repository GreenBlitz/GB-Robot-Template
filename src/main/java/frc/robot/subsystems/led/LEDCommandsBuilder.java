package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class LEDCommandsBuilder {

	private final AddressableLEDWrapper leds;

	protected LEDCommandsBuilder(AddressableLEDWrapper leds) {
		this.leds = leds;
	}

	public Command setPattern(LEDPattern pattern) {
		return leds.asSubsystemCommand(new InstantCommand(() -> leds.setPattern(pattern)), "Set LED pattern");
	}

	public Command blink() {
		return leds.asSubsystemCommand(new RunCommand(leds::applyPattern).beforeStarting(leds::blink), "Blink LEDs");
	}

}
