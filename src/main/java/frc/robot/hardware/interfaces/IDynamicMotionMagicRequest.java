package frc.robot.hardware.interfaces;

import org.wpilib.math.geometry.Angle2d;

public interface IDynamicMotionMagicRequest extends IMotionMagicRequest {

	@Override
	IDynamicMotionMagicRequest withSetPoint(Angle2d setPoint);

	IDynamicMotionMagicRequest withMaxVelocityRPS(Angle2d maxVelocityRPS);

	Angle2d getMaxVelocityRPS();

	IDynamicMotionMagicRequest withMaxAccelerationRPS(Angle2d maxAccelerationRPSSquared);

	Angle2d getMaxAccelerationRPS();

}
