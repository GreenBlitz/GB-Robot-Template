package frc.robot.hardware.phoenix6.request;

import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.VelocityPositionRequest;

import java.util.function.Consumer;

public class Phoenix6VelocityPositionRequest extends Phoenix6FeedForwardRequest implements VelocityPositionRequest {

	private final Consumer<Rotation2d> setVelocity;
	private Rotation2d velocity;
	private Consumer<Rotation2d> setPosition;

	public Phoenix6VelocityPositionRequest(
		Rotation2d position,
		Rotation2d velocity,
		ControlRequest controlRequest,
		Consumer<Rotation2d> setVelocity,
		Consumer<Rotation2d> setPosition,
		Consumer<Double> setFeedForward,
		double defaultArbitraryFeedForward
	) {
		super(position, controlRequest, setPosition, setFeedForward, defaultArbitraryFeedForward);
		this.setVelocity = setVelocity;
		this.velocity = velocity;
	}

	@Override
	public VelocityPositionRequest setVelocity(Rotation2d targetVelocityRPS) {
		this.velocity = targetVelocityRPS;
		setVelocity.accept(targetVelocityRPS);
		return this;
	}

	@Override
	public Rotation2d getVelocityRPS() {
		return velocity;
	}

}
