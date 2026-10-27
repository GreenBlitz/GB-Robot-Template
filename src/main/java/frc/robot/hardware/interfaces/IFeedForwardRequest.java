package frc.robot.hardware.interfaces;

import org.wpilib.math.geometry.Angle2d;

public interface IFeedForwardRequest extends IRequest<Angle2d> {

	IFeedForwardRequest withArbitraryFeedForward(double newArbitraryFeedForward);

	double getArbitraryFeedForward();

}
