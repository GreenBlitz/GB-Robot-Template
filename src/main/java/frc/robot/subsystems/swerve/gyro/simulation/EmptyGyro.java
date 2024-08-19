package frc.robot.subsystems.swerve.gyro.simulation;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.gyro.IGyro;
import frc.robot.subsystems.swerve.gyro.GyroInputsAutoLogged;

public class EmptyGyro implements IGyro {

	@Override
	public void setYaw(Rotation2d heading) {}

	@Override
	public void updateInputs(GyroInputsAutoLogged inputs) {}

}
