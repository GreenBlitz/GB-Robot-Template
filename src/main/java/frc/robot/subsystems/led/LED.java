package frc.robot.subsystems.led;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.GBSubsystem;

import static edu.wpi.first.units.Units.Meters;

public class LED extends GBSubsystem {

	public static LEDConstants ledConstants;
	public static double timeBetweenBlinksSeconds;

	private LEDPattern ledPattern;

	private enum Effect{
		NONE,
		BLINK,
		SCROLL
	}
	private Effect currentEffect;

	private final AddressableLED addressableLED;
	private final AddressableLEDBuffer addressableLEDBuffer;
	private final LEDCommandsBuilder ledCommandsBuilder;

	public LED(LEDConstants ledConstants) {
		super(ledConstants.LOG_PATH());

		this.ledConstants = ledConstants;
		this.timeBetweenBlinksSeconds = ledConstants.DEFAULT_TIME_BETWEEN_BLINKS_SECOND();

		this.addressableLED = new AddressableLED(ledConstants.PORT_NUMBER());
		this.addressableLED.setLength(ledConstants.NUMBER_OF_LEDS_IN_STRIP()); // very expensive function, run ONCE only

		this.addressableLEDBuffer = new AddressableLEDBuffer(ledConstants.NUMBER_OF_LEDS_IN_STRIP());
		this.ledPattern = LEDPattern.solid(ledConstants.DEFAULT_COLOR());
		addressableLED.start();

		this.ledCommandsBuilder = new LEDCommandsBuilder(this);
		setDefaultCommand(new InstantCommand(() -> setPattern(ledPattern)));

		this.currentEffect = Effect.NONE;
	}

	public LEDCommandsBuilder getCommandsBuilder() {
		return ledCommandsBuilder;
	}

	public void setPattern(LEDPattern pattern) {
		ledPattern = pattern;
		currentEffect = Effect.NONE;
	}

	public void setAlternatingColors(Color evenColor, Color oddColor) {
		for (int i = 0; i < addressableLEDBuffer.getLength(); i++) {
			if (i % 2 == 0) {
				addressableLEDBuffer.setLED(i, evenColor);
			} else {
				addressableLEDBuffer.setLED(i, oddColor);
			}
		}

		addressableLED.setData(addressableLEDBuffer);
	}

	public void blinkWithCurrentPattern() {
		currentEffect = Effect.BLINK;
	}

	public void setDualGradient(Color colorA, Color colorB) {
		setPattern(LEDPattern.gradient(LEDPattern.GradientType.kContinuous, colorA, colorB));
	}

	public static void setTimeBetweenBlinksSeconds(double timeBetweenBlinksSeconds) {
		LED.timeBetweenBlinksSeconds = timeBetweenBlinksSeconds;
	}

	public void enableScroll() {
		currentEffect = Effect.SCROLL;
	}

	private void updatePattern() {
		if (currentEffect == Effect.BLINK) {
			ledPattern.blink(
					Units.Seconds.of(ledConstants.DEFAULT_TIME_BETWEEN_BLINKS_SECOND())
			).applyTo(addressableLEDBuffer);

		} else if (currentEffect == Effect.SCROLL) {
			ledPattern.scrollAtAbsoluteSpeed(
					ledConstants.SCROLL_SPEED_PER_METER_PER_SECOND(),
					Meters.of(1.0 / ledConstants.NUMBER_OF_LEDS_PER_METER())
			).applyTo(addressableLEDBuffer);
		} else {
			ledPattern.applyTo(addressableLEDBuffer);
		}
	}

	public void update() {
		super.subsystemPeriodic();
		updatePattern();
	}

}
