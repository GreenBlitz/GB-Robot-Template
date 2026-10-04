package frc.robot.hardware.interfaces;

import org.wpilib.math.geometry.Rotation2d;

public interface VelocityPositionRequest extends IFeedForwardRequest {

	VelocityPositionRequest setVelocity(Rotation2d targetVelocityRPS);

	Rotation2d getVelocityRPS();

}
