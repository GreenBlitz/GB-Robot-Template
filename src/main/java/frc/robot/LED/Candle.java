package frc.robot.LED;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle;

import java.awt.*;

public class Candle implements ILED, ILogicLED {
    private static Candle instance;
    private CANdle caNdle;
    private ColorFlowAnimation colorFlowAnimation;
    private FireAnimation fireAnimation;
    private LarsonAnimation larsonAnimation;
    private RainbowAnimation rainbowAnimation;
    private RgbFadeAnimation rgbFadeAnimation;
    private SingleFadeAnimation singleFadeAnimation;
    private StrobeAnimation strobeAnimation;
    private TwinkleAnimation twinkleAnimation;
    private TwinkleOffAnimation twinkleOffAnimation;


    private Candle() {
        this.caNdle = new CANdle(LEDConstatns.Candle.ID_PORT);
        this.caNdle.configLEDType(CANdle.LEDStripType.RGB);
        this.colorFlowAnimation = new ColorFlowAnimation(0, 255, 255);
        this.fireAnimation = new FireAnimation();
        this.larsonAnimation = new LarsonAnimation(0, 255, 255);
        this.rgbFadeAnimation = new RgbFadeAnimation(0, 255, 255);
        this.singleFadeAnimation = new SingleFadeAnimation(0, 255, 255);
        this.twinkleAnimation = new TwinkleAnimation(0, 255, 255);
        this.twinkleOffAnimation = new TwinkleOffAnimation(0, 255, 255);
    }

    public static Candle getInstance() {
        if (instance == null) {
            instance = new Candle();
        }
        return instance;
    }

    @Override
    public void setColor(Color color) {
        this.caNdle.setLEDs(color.getRed(), color.getGreen(), color.getBlue());
    }

    @Override
    public void turnOff() {
        this.caNdle.setLEDs(0, 0, 0);
    }

    @Override
    public void ColorFlowAnimation() {
        this.caNdle.animate(this.colorFlowAnimation);
    }

    @Override
    public void FireAnimation() {
        this.caNdle.animate(this.fireAnimation);
    }

    @Override
    public void LarsonAnimation() {
        this.caNdle.animate(this.larsonAnimation);
    }

    @Override
    public void RainbowAnimation() {
        this.caNdle.animate(this.rainbowAnimation);
    }

    @Override
    public void RGBFadeAnimation() {
        this.caNdle.animate(this.rgbFadeAnimation);
    }

    @Override
    public void SingleFadeAnimation() {
        this.caNdle.animate(this.singleFadeAnimation);
    }

    @Override
    public void StrobeAnimation() {
        this.caNdle.animate(this.strobeAnimation);
    }

    @Override
    public void TwinkleAnimation() {
        this.caNdle.animate(this.twinkleAnimation);
    }

    @Override
    public void TwinkleOffAnimation() {
        this.caNdle.animate(this.twinkleOffAnimation);
    }

    public void setColorAccordingToState(RobotStates robotStates) {
        switch (robotStates) {
            case STATE1: {
                this.RainbowAnimation();
                break;
            }
            case STATE2:{
                this.ColorFlowAnimation();
                break;
            }
            case STATE3:{
                this.FireAnimation();
                break;
            }
            default:{
                this.turnOff();
                break;
            }
        }
    }
}
