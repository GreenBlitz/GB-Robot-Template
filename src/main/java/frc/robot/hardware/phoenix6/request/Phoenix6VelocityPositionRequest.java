package frc.robot.hardware.phoenix6.request;

import com.ctre.phoenix6.controls.ControlRequest;
import frc.utils.costomUnits.Angle2d;
import frc.robot.hardware.interfaces.VelocityPositionRequest;

import java.util.function.Consumer;

public class Phoenix6VelocityPositionRequest extends Phoenix6FeedForwardRequest implements VelocityPositionRequest {

	private final Consumer<Angle2d> setVelocity;
	private Angle2d velocity;
	private Consumer<Angle2d> setPosition;

	public Phoenix6VelocityPositionRequest(
		Angle2d position,
		Angle2d velocity,
		ControlRequest controlRequest,
		Consumer<Angle2d> setVelocity,
		Consumer<Angle2d> setPosition,
		Consumer<Double> setFeedForward,
		double defaultArbitraryFeedForward
	) {
		super(position, controlRequest, setPosition, setFeedForward, defaultArbitraryFeedForward);
		this.setVelocity = setVelocity;
		this.velocity = velocity;
	}

	@Override
	public VelocityPositionRequest setVelocity(Angle2d targetVelocityRPS) {
		this.velocity = targetVelocityRPS;
		setVelocity.accept(targetVelocityRPS);
		return this;
	}

	@Override
	public Angle2d getVelocityRPS() {
		return velocity;
	}

}
