package frc.robot.hardware.interfaces;

import frc.utils.costomUnits.Angle2d;

public interface IFeedForwardRequest extends IRequest<Angle2d> {

	IFeedForwardRequest withArbitraryFeedForward(double newArbitraryFeedForward);

	double getArbitraryFeedForward();

}
