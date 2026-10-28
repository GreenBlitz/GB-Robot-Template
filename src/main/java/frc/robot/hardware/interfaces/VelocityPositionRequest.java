package frc.robot.hardware.interfaces;

import frc.utils.costomUnits.Angle2d;

public interface VelocityPositionRequest extends IFeedForwardRequest {

	VelocityPositionRequest setVelocity(Angle2d targetVelocityRPS);

	Angle2d getVelocityRPS();

}
