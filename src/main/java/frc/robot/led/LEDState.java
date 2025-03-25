package frc.robot.led;

import com.ctre.phoenix.led.*;

public enum LEDState {

	DISABLE(new StrobeAnimation(250, 60, 0)),
	IDLE(new LarsonAnimation(0, 250, 0, 255, 0.5, LEDConstants.NUMBER_OF_LEDS, LarsonAnimation.BounceMode.Front, 5)),
	READY_TO_COLLECT(new StrobeAnimation(0, 200, 200)),
	HAS_CORAL(new ColorFlowAnimation(200, 0, 200, 250, 1, LEDConstants.NUMBER_OF_LEDS, ColorFlowAnimation.Direction.Forward, 8)),
	TOUCHING_LIMIT_SWITCH(new ColorFlowAnimation(173, 216, 230, 250, 1, LEDConstants.NUMBER_OF_LEDS, ColorFlowAnimation.Direction.Forward, 8)),
	MOVE_TO_POSE(new ColorFlowAnimation(200, 200, 0, 250, 1, LEDConstants.NUMBER_OF_LEDS, ColorFlowAnimation.Direction.Forward, 8)),
	IN_POSITION_TO_OPEN_ELEVATOR(new StrobeAnimation(200, 200, 0)),
	OPENING_SUPERSTRUCTURE(new ColorFlowAnimation(0, 200, 0, 250, 1, LEDConstants.NUMBER_OF_LEDS, ColorFlowAnimation.Direction.Forward, 8)),
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
