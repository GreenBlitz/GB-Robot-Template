package frc.robot.hardware.interfaces;

import edu.wpi.first.math.geometry.Rotation2d;

public interface VelocityPositionRequest extends IFeedForwardRequest {

	VelocityPositionRequest setVelocity(Rotation2d targetVelocityRPS);

	Rotation2d getVelocityRPS();

}
