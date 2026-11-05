package frc.robot.hardware.interfaces;


import frc.utils.costomUnits.Angle2d;

public interface IAngleEncoder extends IDevice {

	void setPosition(Angle2d position);

}
