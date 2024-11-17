package frc.robot.hardware.interfaces;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IAngleEncoder extends IDevice {

	void setPosition(Rotation2d position);

}
