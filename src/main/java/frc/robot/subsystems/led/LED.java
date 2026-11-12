package frc.robot.subsystems.led;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.GBSubsystem;

import static edu.wpi.first.units.Units.Meters;

public class LED extends GBSubsystem {

	public static LEDConstants ledConstants;
	public static double timeBetweenBlinksSeconds;
	private final AddressableLED addressableLED;
	private final AddressableLEDBuffer addressableLEDBuffer;
	private LEDPattern ledPattern;
	private LEDPattern basePattern;
	private final LEDCommandsBuilder ledCommandsBuilder;

	public LED(LEDConstants ledConstants) {
		super(ledConstants.LOG_PATH());

		this.ledConstants = ledConstants;
		this.timeBetweenBlinksSeconds = ledConstants.DEFAULT_TIME_BETWEEN_BLINKS_SECOND();

		this.addressableLED = new AddressableLED(ledConstants.PORT_NUMBER());
		this.addressableLED.setLength(ledConstants.NUMBER_OF_LEDS_IN_STRIP()); // very expensive function, run ONCE only

		this.addressableLEDBuffer = new AddressableLEDBuffer(ledConstants.NUMBER_OF_LEDS_IN_STRIP());
		this.ledPattern = LEDPattern.solid(ledConstants.DEFAULT_COLOR());
		this.basePattern = ledPattern;
		addressableLED.start();

		this.ledCommandsBuilder = new LEDCommandsBuilder(this);
	}

	public LEDCommandsBuilder getCommandsBuilder() {
		return ledCommandsBuilder;
	}

	public LEDPattern getBasePattern() {
		return basePattern;
	}

	public void setGenericPattern(LEDPattern pattern) {
		ledPattern = pattern;
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
		basePattern = ledPattern;
		setGenericPattern(ledPattern.blink(Units.Seconds.of(ledConstants.DEFAULT_TIME_BETWEEN_BLINKS_SECOND())));
	}

	public void setDualGradient(Color colorA, Color colorB) {
		setGenericPattern(LEDPattern.gradient(LEDPattern.GradientType.kContinuous, colorA, colorB));
	}

	public static void setTimeBetweenBlinksSeconds(double timeBetweenBlinksSeconds) {
		LED.timeBetweenBlinksSeconds = timeBetweenBlinksSeconds;
	}

	public void enableScroll(LinearVelocity linearVelocity) {
		basePattern = ledPattern;
		setGenericPattern(ledPattern.scrollAtAbsoluteSpeed(linearVelocity, Meters.of(1 / ledConstants.NUMBER_OF_LEDS_PER_METER())));
	}

	private void applyPattern(){
		ledPattern.applyTo(addressableLEDBuffer);
		addressableLED.setData(addressableLEDBuffer);
	}

	@Override
	public void subsystemPeriodic() {
		super.subsystemPeriodic();
		applyPattern();
	}

}
