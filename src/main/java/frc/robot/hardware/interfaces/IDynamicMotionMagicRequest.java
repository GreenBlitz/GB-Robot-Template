package frc.robot.hardware.interfaces;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IDynamicMotionMagicRequest extends IMotionMagicRequest {

	@Override
	IDynamicMotionMagicRequest withSetPoint(Rotation2d setPoint);

	IDynamicMotionMagicRequest withMaxVelocityRPS(Rotation2d maxVelocityRPS);

	Rotation2d getMaxVelocityRPS();

	IDynamicMotionMagicRequest withMaxAccelerationRPS(Rotation2d maxAccelerationRPSSquared);

	Rotation2d getMaxAccelerationRPS();

}
