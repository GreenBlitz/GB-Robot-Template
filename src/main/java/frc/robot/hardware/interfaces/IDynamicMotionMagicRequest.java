package frc.robot.hardware.interfaces;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IDynamicMotionMagicRequest extends IMotionMagicRequest {

	@Override
	IDynamicMotionMagicRequest withSetPoint(Rotation2d setPoint);

	IDynamicMotionMagicRequest withMaxVelocityRotation2dPerSecond(Rotation2d maxVelocityRotation2dPerSecond);

	Rotation2d getMaxVelocityRotation2dPerSecond();

	IDynamicMotionMagicRequest withMaxAccelerationRotation2dPerSecondSquared(Rotation2d maxAccelerationRotation2dPerSecondSquared);

	Rotation2d getMaxAccelerationRotation2dPerSecondSquared();

}
