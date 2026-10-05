package frc.robot.subsystems.led;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.GBSubsystem;

public class AddressableLEDWrapper extends GBSubsystem {

	private static final LEDConstants ledConstants = new LEDConstants(4, 50, 0.5);
	private final AddressableLED addressableLED;
	private final AddressableLEDBuffer addressableLEDBuffer;
	private LEDPattern ledPattern;
	private final LEDCommandsBuilder ledCommandsBuilder;

	public AddressableLEDWrapper(String logPath) {
		super(logPath);

		this.addressableLED = new AddressableLED(ledConstants.portNum());
		this.addressableLED.setLength(ledConstants.numOfLEDsInStrip()); // very expensive function, run ONCE only

		this.addressableLEDBuffer = new AddressableLEDBuffer(ledConstants.numOfLEDsInStrip());
		this.ledPattern = LEDPattern.solid(Color.kBlack);
		applyPattern();
		addressableLED.start();

		this.ledCommandsBuilder = new LEDCommandsBuilder(this);
	}

	public LEDCommandsBuilder getLedCommandsBuilder() {
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

	public void blink() {
		ledPattern = ledPattern.blink(Units.Seconds.of(ledConstants.secondsBetweenBlinks()));
	}

	void applyPattern() {
		ledPattern.applyTo(addressableLEDBuffer);
		addressableLED.setData(addressableLEDBuffer);
	}

}
