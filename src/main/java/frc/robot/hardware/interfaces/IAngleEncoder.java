package frc.robot.hardware.interfaces;

import org.wpilib.math.geometry.Angle2d;

public interface IAngleEncoder extends IDevice {

	void setPosition(Angle2d position);

}
