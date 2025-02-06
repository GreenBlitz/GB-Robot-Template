package frc.robot.led;

import com.ctre.phoenix.led.*;

public enum LEDState {
    TAG_VISIBLE(new TwinkleAnimation(255, 0, 0)),
    HAS_CORAL(new TwinkleAnimation(0, 255, 0)),
    ;

    public final Animation animation;

    LEDState(Animation animation) {
        this.animation = animation;
    }
}
