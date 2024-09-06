package frc.robot.subsystems.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.gyro.GyroInputsAutoLogged;

public class EmptyThreadedGyro implements IThreadedGyro {

	@Override
	public void setYaw(Rotation2d yaw) {}

	@Override
	public void updateInputs(GyroInputsAutoLogged inputs) {}

	@Override
	public void updateInputs(GyroThreadInputsAutoLogged inputs) {}

}
