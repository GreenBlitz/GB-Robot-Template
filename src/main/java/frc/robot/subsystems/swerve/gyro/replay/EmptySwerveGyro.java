package frc.robot.subsystems.swerve.gyro.replay;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.gyro.ISwerveGyro;
import frc.robot.subsystems.swerve.gyro.SwerveGyroInputsAutoLogged;

public class EmptySwerveGyro implements ISwerveGyro {

	@Override
	public void setYaw(Rotation2d heading) {}

	@Override
	public void updateInputs(SwerveGyroInputsAutoLogged inputs) {}

}
