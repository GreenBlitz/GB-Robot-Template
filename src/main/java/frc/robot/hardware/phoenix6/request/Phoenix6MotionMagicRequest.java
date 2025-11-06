package frc.robot.hardware.phoenix6.request;

import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.IMotionMagicRequest;

import java.util.function.Consumer;

public class Phoenix6MotionMagicRequest extends Phoenix6FeedForwardRequest implements IMotionMagicRequest {

	Phoenix6MotionMagicRequest(
		Rotation2d defaultSetPoint,
		ControlRequest controlRequest,
		Consumer<Rotation2d> setSetPoint,
		Consumer<Double> setFeedForward,
		double defaultArbitraryFeedForward
	) {
		super(defaultSetPoint, controlRequest, setSetPoint, setFeedForward, defaultArbitraryFeedForward);
	}

}
