package frc.robot.hardware.phoenix6.request;

import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

public class Phoenix6FeedForwardRequest extends Phoenix6Request<Rotation2d> {

	private final Consumer<Double> setFeedFroward;
	private final BooleanSupplier applyFeedForward;
	private  double arbitraryFeedForward;

	Phoenix6FeedForwardRequest(
			Rotation2d defaultSetPoint,
			ControlRequest controlRequest,
			Consumer<Rotation2d> setSetPoint,
			Consumer<Double> setFeedFroward,
			BooleanSupplier applyFeedForward,
			double defaultArbitraryFeedForward
	) {
		super(defaultSetPoint, controlRequest, setSetPoint);
		this.setFeedFroward = setFeedFroward;
		this.applyFeedForward = applyFeedForward;
		this.arbitraryFeedForward = defaultArbitraryFeedForward;
	}

	public Phoenix6FeedForwardRequest withArbitraryFeedForward(double newFeedForward) {
		setFeedFroward.accept(newFeedForward);
		this.arbitraryFeedForward = newFeedForward;
		return this;
	}

	public void ApplyFeedForward(){
		if (applyFeedForward.getAsBoolean()){
			setFeedFroward.accept(arbitraryFeedForward);
		}
		else {
			setFeedFroward.accept(0.0);
		}
	}

}
