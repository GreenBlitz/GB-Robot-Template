package frc.robot.hardware.interfaces;

import frc.utils.angle2d.Angle2d;

public interface VelocityPositionRequest extends IFeedForwardRequest {

	VelocityPositionRequest setVelocity(Angle2d targetVelocityAPS);

	Angle2d getVelocityAPS();

}
