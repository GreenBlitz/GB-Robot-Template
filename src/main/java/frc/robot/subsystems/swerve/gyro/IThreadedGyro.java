package frc.robot.subsystems.swerve.gyro;

import frc.robot.hardware.gyro.IGyro;

public interface IThreadedGyro extends IGyro {

	void updateInputs(GyroThreadInputsAutoLogged gyroThreadInputs);

}
