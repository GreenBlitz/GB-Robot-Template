package frc.robot.hardware.gyro;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IGyro {

	void setYaw(Rotation2d yaw);

	void updateInputs(GyroInputsAutoLogged inputs);

}
