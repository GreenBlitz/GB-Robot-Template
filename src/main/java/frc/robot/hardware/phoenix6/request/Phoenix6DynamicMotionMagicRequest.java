package frc.robot.hardware.phoenix6.request;

import com.ctre.phoenix6.controls.ControlRequest;
import frc.utils.costomUnits.Angle2d;
import frc.robot.hardware.interfaces.IDynamicMotionMagicRequest;

import java.util.function.Consumer;

public class Phoenix6DynamicMotionMagicRequest extends Phoenix6MotionMagicRequest implements IDynamicMotionMagicRequest {

	private final Consumer<Angle2d> setMaxVelocity;
	private Angle2d maxVelocity;

	private final Consumer<Angle2d> setMaxAcceleration;
	private Angle2d maxAcceleration;

	Phoenix6DynamicMotionMagicRequest(
		Angle2d defaultSetPoint,
		ControlRequest controlRequest,
		Consumer<Angle2d> setSetPoint,
		Consumer<Double> setFeedForward,
		Consumer<Angle2d> setMaxVelocity,
		Consumer<Angle2d> setMaxAcceleration,
		double defaultArbitraryFeedForward
	) {
		super(defaultSetPoint, controlRequest, setSetPoint, setFeedForward, defaultArbitraryFeedForward);
		this.setMaxVelocity = setMaxVelocity;
		this.setMaxAcceleration = setMaxAcceleration;
		maxVelocity = new Angle2d();
		maxAcceleration = new Angle2d();
	}

	@Override
	public Phoenix6DynamicMotionMagicRequest withSetPoint(Angle2d setPoint) {
		super.withSetPoint(setPoint);
		return this;
	}

	@Override
	public IDynamicMotionMagicRequest withMaxVelocityRPS(Angle2d maxVelocityRPS) {
		maxVelocity = maxVelocityRPS;
		setMaxVelocity.accept(maxVelocity);
		return this;
	}

	@Override
	public Angle2d getMaxVelocityRPS() {
		return maxVelocity;
	}

	@Override
	public IDynamicMotionMagicRequest withMaxAccelerationRPS(Angle2d maxAccelerationRPS) {
		maxAcceleration = maxAccelerationRPS;
		setMaxAcceleration.accept(maxAcceleration);
		return this;
	}

	@Override
	public Angle2d getMaxAccelerationRPS() {
		return maxAcceleration;
	}

}
