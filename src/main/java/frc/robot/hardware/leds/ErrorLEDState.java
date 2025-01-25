package frc.robot.hardware.leds;

import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public enum ErrorLEDState {

    NONE(LEDPattern.kOff),
    WARNING(LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kLightYellow, Color.kYellow, Color.kLightGoldenrodYellow)),
    ERROR(LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kOrangeRed, Color.kRed, Color.kMediumVioletRed, Color.kDarkRed));

    private final LEDPattern ledPattern;

    ErrorLEDState(LEDPattern ledPattern) {
        this.ledPattern = ledPattern;
    }

    public void applyState(WPILEDsManager manager, AddressableLEDBufferView section) {
        manager.applyPatternOnSection(ledPattern, section);
    }

}
