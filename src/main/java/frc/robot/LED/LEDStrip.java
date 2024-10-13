package frc.robot.LED;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import java.awt.*;

public class LEDStrip implements ILED {

    private AddressableLED addressableLED;

    private AddressableLEDBuffer addressableLEDBuffer;

    public LEDStrip() {
        this.addressableLED = new AddressableLED(LEDConstants.LEDStrip.LED_PORT);
        this.addressableLEDBuffer = new AddressableLEDBuffer(LEDConstants.LEDStrip.LED_LENGTH);
        addressableLED.setLength(addressableLEDBuffer.getLength());
        addressableLED.setData(addressableLEDBuffer);
        this.addressableLED.start();
    }

    @Override
    public void setSingleLEDColor(Color color, int index) {
        addressableLEDBuffer.setRGB(index, color.getRed(), color.getGreen(), color.getBlue());
        this.update();

    }

    @Override
    public void setSectionColor(Color color, int startIndex, int endIndex) {
        for (int i = startIndex; i < endIndex; i++) {
            setSingleLEDColor(color, i);
        }
    }

    @Override
    public void singleLEDTurnOff(int index) {
        setSingleLEDColor(Color.BLACK, index);
    }

    @Override
    public void update() {
        addressableLED.setData(this.addressableLEDBuffer);
    }

    @Override
    public void sectionTurnOff(int startIndex, int endIndex) {
        setSectionColor(Color.BLACK, 0, LEDConstants.LEDStrip.LED_LENGTH);
    }


}
