package frc.robot.led;

import com.ctre.phoenix.led.*;

public enum LEDState {
    IDLE(new FireAnimation()),
    DISABLE(new RainbowAnimation()),
    READY_TO_COLLECT(new StrobeAnimation(0, 200, 200)),
    HAS_CORAL(new StrobeAnimation(200, 0, 200)),
    START_AIM_ASSIST(new SingleFadeAnimation(200, 0, 0)),
    IS_IN_POSITION_TO_OPEN_ELEVATOR(new StrobeAnimation(255, 0, 0)),
//    OPENING_SUPERSTRUCTURE(new SingleFadeAnimation(200, 200, 0)),
    SUPERSTRUCTURE_IN_POSITION(new StrobeAnimation(200, 200, 0)),
    IN_POSITION_TO_SCORE(new StrobeAnimation(0, 200, 0)),
    ;

    public final Animation animation;

    LEDState(Animation animation) {
        this.animation = animation;
    }
}
