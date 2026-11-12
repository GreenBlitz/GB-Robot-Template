package frc.robot.hardware.interfaces;

import frc.utils.angle2d.Angle2d;

public interface IDynamicMotionMagicRequest extends IMotionMagicRequest {

	@Override
	IDynamicMotionMagicRequest withSetPoint(Angle2d setPoint);

	IDynamicMotionMagicRequest withMaxVelocityAPS(Angle2d maxVelocityAPS);

	Angle2d getMaxVelocityAPS();

	IDynamicMotionMagicRequest withMaxAccelerationAPS(Angle2d maxAccelerationAPSSquared);

	Angle2d getMaxAccelerationAPS();

}
