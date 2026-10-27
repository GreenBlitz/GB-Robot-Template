package frc.robot.hardware.interfaces;

import org.wpilib.math.geometry.Angle2d;

public interface VelocityPositionRequest extends IFeedForwardRequest {

	VelocityPositionRequest setVelocity(Angle2d targetVelocityRPS);

	Angle2d getVelocityRPS();

}
