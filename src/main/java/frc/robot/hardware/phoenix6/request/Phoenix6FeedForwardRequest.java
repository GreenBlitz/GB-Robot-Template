package frc.robot.hardware.phoenix6.request;

import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.function.Consumer;

public class Phoenix6FeedForwardRequest extends Phoenix6Request<Rotation2d> {

	private final Consumer<Double> setFeedFroward;
	private double feedFroward;

	Phoenix6FeedForwardRequest(
		Rotation2d defaultSetPoint,
		ControlRequest controlRequest,
		Consumer<Rotation2d> setSetPoint,
		Consumer<Double> setFeedFroward
	) {
		super(defaultSetPoint, controlRequest, setSetPoint);
		this.setFeedFroward = setFeedFroward;
		this.feedFroward = 0;
	}

	public Phoenix6FeedForwardRequest withArbitraryFeedForward(double newFeedForward) {
		setFeedFroward.accept(newFeedForward);
		this.feedFroward = newFeedForward;
		return this;
	}

	public double getFeedForward() {
		return feedFroward;
	}

}
