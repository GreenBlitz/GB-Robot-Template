package frc.robot.led;

import com.ctre.phoenix.led.*;

public enum LEDState {

	DISABLE(new StrobeAnimation(250, 60, 0)),
	IDLE(new LarsonAnimation(0, 250, 0, 255, 0.5, LEDConstants.NUMBER_OF_LEDS, LarsonAnimation.BounceMode.Front, 5)),
	READY_TO_COLLECT(new StrobeAnimation(0, 200, 200)),
	HAS_CORAL(new ColorFlowAnimation(200, 0, 200, 250, 2, 100, ColorFlowAnimation.Direction.Forward, 8)),
	START_AIM_ASSIST(new SingleFadeAnimation(200, 0, 0)),
	IS_IN_POSITION_TO_OPEN_ELEVATOR(new StrobeAnimation(255, 0, 0)),
	SUPERSTRUCTURE_IN_POSITION(new StrobeAnimation(200, 200, 0)),
	IN_POSITION_TO_SCORE(new StrobeAnimation(0, 200, 0)),
	INTAKE(new StrobeAnimation(200, 0, 200));

	private final Animation animation;

	LEDState(Animation animation) {
		this.animation = animation;
	}

	public Animation getAnimation() {
		return animation;
	}

}
