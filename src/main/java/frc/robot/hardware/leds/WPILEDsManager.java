package frc.robot.hardware.leds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import java.util.HashMap;

import static edu.wpi.first.units.Units.Meters;

public class WPILEDsManager extends AddressableLED {

	private final AddressableLEDBuffer ledBuffer;
	private final Distance metersPerLed;
	private final HashMap<AddressableLEDBufferView, LEDPattern> patternsPerBufferView;

	/**
	 * Constructs a new driver for a specific port.
	 *
	 * @param port the output port to use (Must be a PWM header, not on MXP)
	 */
	public WPILEDsManager(int port, int lengthInLEDs, double ledsPerMeter) {
		super(port);
		super.setLength(lengthInLEDs);

		this.ledBuffer = new AddressableLEDBuffer(lengthInLEDs);
		this.metersPerLed = Meters.of(1 / ledsPerMeter);
		this.patternsPerBufferView = new HashMap<>();

		super.setData(ledBuffer);
		super.start();
	}

	public Distance getMetersPerLED() {
		return metersPerLed;
	}

	public int getNumberOfLEDs() {
		return ledBuffer.getLength();
	}

	public AddressableLEDBufferView createSection(int startingIndex, int endingIndex) {
		AddressableLEDBufferView bufferView = ledBuffer.createView(startingIndex, endingIndex);
		applyPatternOnSection(LEDPattern.kOff, bufferView);

		return bufferView;
	}

	public void run() {
		applyPatternsOnBuffers();
		super.setData(ledBuffer);
	}

	private void applyPatternsOnBuffers() {
		for (AddressableLEDBufferView buffer : patternsPerBufferView.keySet()) {
			patternsPerBufferView.get(buffer).applyTo(buffer);
		}
	}


	public void applyPatternOnSection(LEDPattern pattern, AddressableLEDBufferView buffer) {
		patternsPerBufferView.put(buffer, pattern);
	}

	public void applyPatternOnAll(LEDPattern pattern) {
		for (AddressableLEDBufferView buffer : patternsPerBufferView.keySet()) {
			applyPatternOnSection(pattern, buffer);
		}
	}

	public void turnOffAll() {
		applyPatternOnAll(LEDPattern.kOff);
	}

}
