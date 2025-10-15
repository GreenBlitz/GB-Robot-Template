package frc.robot.hardware.interfaces;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IFeedForwardRequest extends IRequest<Rotation2d> {

	IFeedForwardRequest withArbitraryFeedForward(double newArbitraryFeedForward);

	double getArbitraryFeedForward();

}
