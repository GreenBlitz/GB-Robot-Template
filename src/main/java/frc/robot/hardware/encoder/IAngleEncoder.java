package frc.robot.hardware.encoder;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.IDevice;
import frc.robot.hardware.signal.InputSignal;

public interface IAngleEncoder extends IDevice {

	void setPosition(Rotation2d position);

}
