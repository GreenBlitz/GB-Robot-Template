package frc.robot.hardware.phoenix6.request;

import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.function.Consumer;

public class Phoenix6FeedForwardRequest extends Phoenix6Request<Rotation2d> {

	private final Consumer<Double> setArbitraryFeedForward;
	private double arbitraryFeedForward;

	Phoenix6FeedForwardRequest(
		Rotation2d defaultSetPoint,
		ControlRequest controlRequest,
		Consumer<Rotation2d> setSetPoint,
		Consumer<Double> setFeedForward,
		double defaultArbitraryFeedForward
	) {
		super(defaultSetPoint, controlRequest, setSetPoint);
		this.setArbitraryFeedForward = setFeedForward;
		this.arbitraryFeedForward = defaultArbitraryFeedForward;
	}

	public Phoenix6FeedForwardRequest withArbitraryFeedForward(double newArbitraryFeedForward) {
		setArbitraryFeedForward.accept(newArbitraryFeedForward);
		this.arbitraryFeedForward = newArbitraryFeedForward;
		return this;
	}

	public double getArbitraryFeedForward() {
		return arbitraryFeedForward;
	}

}
