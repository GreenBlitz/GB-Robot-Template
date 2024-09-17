package frc.robot.hardware.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.IDevice;

public interface IGyro extends IDevice {

	void setYaw(Rotation2d yaw);

	void setPitch(Rotation2d pitch);

	void setRoll(Rotation2d roll);

}
