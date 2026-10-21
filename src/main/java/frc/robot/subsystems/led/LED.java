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
	private final AddressableLED addressableLED;
	private final AddressableLEDBuffer addressableLEDBuffer;
	private LEDPattern ledPattern;
	private final LEDCommandsBuilder ledCommandsBuilder;

	public LED(String logPath, LEDConstants ledConstants) {
		super(logPath);

		this.ledConstants = ledConstants;

		this.addressableLED = new AddressableLED(ledConstants.portNum());
		this.addressableLED.setLength(ledConstants.numOfLEDsInStrip()); // very expensive function, run ONCE only

		this.addressableLEDBuffer = new AddressableLEDBuffer(ledConstants.numOfLEDsInStrip());
		this.ledPattern = LEDPattern.solid(Color.kBlack);
		applyPattern();
		addressableLED.start();

		this.ledCommandsBuilder = new LEDCommandsBuilder(this);
	}

	public LEDCommandsBuilder getCommandsBuilder() {
		return ledCommandsBuilder;
	}

	public void setPattern(LEDPattern pattern) {
		ledPattern = pattern;
		applyPattern();
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
		ledPattern = ledPattern.blink(Units.Seconds.of(ledConstants.secondsBetweenBlinks()));
	}

	public void setDualGradient(Color colorA, Color colorB) {
		setPattern(LEDPattern.gradient(LEDPattern.GradientType.kContinuous, colorA, colorB));
	}

	public void enableScroll(LinearVelocity linearVelocity) {
		setPattern(ledPattern.scrollAtAbsoluteSpeed(linearVelocity, Meters.of(1 / ledConstants.ledAmountPerMeter())));
	}

	void applyPattern() {
		ledPattern.applyTo(addressableLEDBuffer);
		addressableLED.setData(addressableLEDBuffer);
	}

}
