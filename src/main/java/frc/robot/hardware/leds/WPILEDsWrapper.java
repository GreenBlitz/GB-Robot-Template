package frc.robot.hardware.leds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import java.util.HashMap;

import static edu.wpi.first.units.Units.Meters;

public class WPILEDsWrapper extends AddressableLED {


	private AddressableLEDBuffer ledBuffer;
	private Distance metersPerLed;
	private HashMap<AddressableLEDBufferView, LEDPattern> patternsPerBufferView;

	/**
	 * Constructs a new driver for a specific port.
	 *
	 * @param port the output port to use (Must be a PWM header, not on MXP)
	 */
	public WPILEDsWrapper(int port, int lengthInLEDs, double ledsPerMeter) {
		super(port);
		super.setLength(lengthInLEDs);

		this.ledBuffer = new AddressableLEDBuffer(lengthInLEDs);
		this.metersPerLed = Meters.of(1 / ledsPerMeter);
		this.patternsPerBufferView = new HashMap<>();

		super.setData(ledBuffer);
		super.start();
	}

	public void applyPatternsToBuffer(LEDPattern pattern, AddressableLEDBufferView buffer) {
		patternsPerBufferView.put(buffer, pattern);
	}

	public AddressableLEDBufferView createBuffer(int startingIndex, int endingIndex) {
		AddressableLEDBufferView bufferView = ledBuffer.createView(startingIndex, endingIndex);
		applyPatternsToBuffer(LEDPattern.kOff, bufferView);

		return bufferView;
	}

	private void applyPatternOnAll(LEDPattern pattern) {
		for (AddressableLEDBufferView buffer : patternsPerBufferView.keySet()) {
			applyPatternsToBuffer(pattern, buffer);
		}
	}

	public void turnAllOff() {
		this.applyPatternOnAll(LEDPattern.kOff);
	}

	public void applyPatternToAll(LEDPattern pattern) {
		this.applyPatternOnAll(pattern);
	}

	public void periodic() {
		applyPatternsToBuffers();
		super.setData(ledBuffer);
	}

	private void applyPatternsToBuffers() {
		for (AddressableLEDBufferView buffer : patternsPerBufferView.keySet()) {
			patternsPerBufferView.get(buffer).applyTo(buffer);
		}
	}

	public Distance getMetersPerLed() {
		return metersPerLed;
	}

	public int length() {
		return ledBuffer.getLength();
	}

}
