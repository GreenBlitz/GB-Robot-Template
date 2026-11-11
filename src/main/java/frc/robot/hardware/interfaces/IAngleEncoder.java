package frc.robot.hardware.interfaces;

import frc.utils.angle2d.Angle2d;

public interface IAngleEncoder extends IDevice {

	void setPosition(Angle2d position);

}
