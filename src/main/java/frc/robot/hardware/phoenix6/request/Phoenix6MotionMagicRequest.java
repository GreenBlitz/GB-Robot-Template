package frc.robot.hardware.phoenix6.request;

import com.ctre.phoenix6.controls.ControlRequest;
import frc.utils.costomUnits.Angle2d;
import frc.robot.hardware.interfaces.IMotionMagicRequest;

import java.util.function.Consumer;

public class Phoenix6MotionMagicRequest extends Phoenix6FeedForwardRequest implements IMotionMagicRequest {

	Phoenix6MotionMagicRequest(
		Angle2d defaultSetPoint,
		ControlRequest controlRequest,
		Consumer<Angle2d> setSetPoint,
		Consumer<Double> setFeedForward,
		double defaultArbitraryFeedForward
	) {
		super(defaultSetPoint, controlRequest, setSetPoint, setFeedForward, defaultArbitraryFeedForward);
	}

}
