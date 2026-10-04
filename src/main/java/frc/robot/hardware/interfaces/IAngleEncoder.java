package frc.robot.hardware.interfaces;

import org.wpilib.math.geometry.Rotation2d;

public interface IAngleEncoder extends IDevice {

	void setPosition(Rotation2d position);

}
