package frc.robot.hardware.phoenix6.request;

import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.function.Consumer;

public class Phoenix6FeedForwardRequest extends Phoenix6Request<Rotation2d> {

	private final Consumer<Double> setFeedFroward;
	private double arbitraryFeedFroward;

	Phoenix6FeedForwardRequest(
		Rotation2d defaultSetPoint,
		ControlRequest controlRequest,
		Consumer<Rotation2d> setSetPoint,
		Consumer<Double> setFeedFroward,
		double defaultArbitraryFeedFroward
	) {
		super(defaultSetPoint, controlRequest, setSetPoint);
		this.setFeedFroward = setFeedFroward;
		this.arbitraryFeedFroward = defaultArbitraryFeedFroward;
	}

	public Phoenix6FeedForwardRequest withArbitraryFeedForward(double newArbitraryFeedForward) {
		setFeedFroward.accept(newArbitraryFeedForward);
		this.arbitraryFeedFroward = newArbitraryFeedForward;
		return this;
	}

	public double getArbitraryFeedForward() {
		return arbitraryFeedFroward;
	}

}
