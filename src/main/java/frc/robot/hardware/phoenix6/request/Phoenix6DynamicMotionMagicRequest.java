package frc.robot.hardware.phoenix6.request;

import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.IDynamicMotionMagicRequest;

import java.util.function.Consumer;

public class Phoenix6DynamicMotionMagicRequest extends Phoenix6MotionMagicRequest implements IDynamicMotionMagicRequest {

	private final Consumer<Rotation2d> setMaxVelocity;
	private Rotation2d maxVelocity;

	private final Consumer<Rotation2d> setMaxAcceleration;
	private Rotation2d maxAcceleration;

	Phoenix6DynamicMotionMagicRequest(
		Rotation2d defaultSetPoint,
		ControlRequest controlRequest,
		Consumer<Rotation2d> setSetPoint,
		Consumer<Double> setFeedForward,
		Consumer<Rotation2d> setMaxVelocity,
		Consumer<Rotation2d> setMaxAcceleration,
		double defaultArbitraryFeedForward
	) {
		super(defaultSetPoint, controlRequest, setSetPoint, setFeedForward, defaultArbitraryFeedForward);
		this.setMaxVelocity = setMaxVelocity;
		this.setMaxAcceleration = setMaxAcceleration;
		maxVelocity = new Rotation2d();
		maxAcceleration = new Rotation2d();
	}

	@Override
	public Phoenix6DynamicMotionMagicRequest withSetPoint(Rotation2d setPoint) {
		super.withSetPoint(setPoint);
		return this;
	}

	@Override
	public IDynamicMotionMagicRequest withMaxVelocityRotation2dPerSecond(Rotation2d maxVelocityRotation2dPerSecond) {
		maxVelocity = maxVelocityRotation2dPerSecond;
		setMaxVelocity.accept(maxVelocity);
		return this;
	}

	@Override
	public Rotation2d getMaxVelocityRotation2dPerSecond() {
		return maxVelocity;
	}

	@Override
	public IDynamicMotionMagicRequest withMaxAccelerationRotation2dPerSecondSquared(Rotation2d maxAccelerationRotation2dPerSecondSquared) {
		maxAcceleration = maxAccelerationRotation2dPerSecondSquared;
		setMaxAcceleration.accept(maxAcceleration);
		return this;
	}

	@Override
	public Rotation2d getMaxAccelerationRotation2dPerSecondSquared() {
		return maxAcceleration;
	}

}
