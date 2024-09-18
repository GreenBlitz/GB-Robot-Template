package frc.robot.LED;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import java.awt.*;

public class AddressableLEDGB implements ILED {
    private AddressableLED addressableLED;
    private AddressableLEDBuffer addressableLEDBuffer;

    public AddressableLEDGB() {
        addressableLED = new AddressableLED(LEDConstatns.LEDStrip.LED_PORT);
        addressableLEDBuffer = new AddressableLEDBuffer(LEDConstatns.LEDStrip.STRIP_LENGTH);
        addressableLED.setLength(addressableLEDBuffer.getLength());
        addressableLED.setData(addressableLEDBuffer);
        addressableLED.start();
    }

    @Override
    public void setColor(Color color) {
        setColorForAllLEDs(color);
    }


    @Override
    public void turnOff() {
        setColor(Color.WHITE);
    }
    private void setColorForASingleLed(int i, Color color){
        addressableLEDBuffer.setRGB(i, color.getRed(), color.getGreen(),color.getBlue());
        addressableLED.setData(addressableLEDBuffer);
    }
    private void setColorForAllLEDs(Color color){
        for (int i = 0; i<this.addressableLEDBuffer.getLength(); i++){
            setColorForASingleLed(i,color);
        }
    }
}
