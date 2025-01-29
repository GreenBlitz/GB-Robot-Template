package frc.robot.hardware.phoenix6.request;

import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.IRequest;

import java.util.function.Consumer;

public class Phoenix6FeedForwardRequest implements IRequest<Rotation2d> {

	private final ControlRequest controlRequest;
	private final Consumer<Rotation2d> setSetPoint;
	private final Consumer<Double> setFeedFroward;
	private Rotation2d setPoint;

	Phoenix6FeedForwardRequest(
		Rotation2d defaultSetPoint,
		ControlRequest controlRequest,
		Consumer<Rotation2d> setSetPoint,
		Consumer<Double> setFeedFroward
	) {
		this.setPoint = defaultSetPoint;
		this.controlRequest = controlRequest;
		this.setSetPoint = setSetPoint;
		this.setFeedFroward = setFeedFroward;
	}

	@Override
	public Phoenix6FeedForwardRequest withSetPoint(Rotation2d setPoint) {
		setSetPoint.accept(setPoint);
		this.setPoint = setPoint;
		return this;
	}

	public Phoenix6FeedForwardRequest withArbitraryFeedForward(double newFeedForward) {
		setFeedFroward.accept(newFeedForward);
		return this;
	}


	@Override
	public Rotation2d getSetPoint() {
		return setPoint;
	}

	public ControlRequest getControlRequest() {
		return controlRequest;
	}

}
