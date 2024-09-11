package frc.robot.LED;

import com.ctre.phoenix.led.*;

public interface ILogicLED {
    void ColorFlowAnimation();
    void FireAnimation();
    void LarsonAnimation();
    void RainbowAnimation();
    void RGBFadeAnimation();
    void SingleFadeAnimation();
    void StrobeAnimation();
    void TwinkleAnimation();
    void TwinkleOffAnimation();
}
