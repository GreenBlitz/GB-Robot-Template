package frc.robot.hardware.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import frc.robot.subsystems.GBSubsystem;
import org.littletonrobotics.junction.Logger;

public class WPILEDsWrapper extends AddressableLED {


    private AddressableLEDBuffer ledBuffer;
    /**
     * Constructs a new driver for a specific port.
     *
     * @param port the output port to use (Must be a PWM header, not on MXP)
     */
    public WPILEDsWrapper(int port, int lengthInLEDs) {
        super(port);
        super.setLength(lengthInLEDs);

        this.ledBuffer = new AddressableLEDBuffer(lengthInLEDs);

        super.setData(ledBuffer);
        super.start();
    }

    public void applyPatternToBuffer(LEDPattern pattern){
        pattern.applyTo(ledBuffer);
    }

    private void applyBufferToLEDs(){
        super.setData(ledBuffer);
    }

    public void periodic (){
        applyBufferToLEDs();
    }





}
