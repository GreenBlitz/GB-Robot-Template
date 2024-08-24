package frc.robot.subsystems.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation2d;

public class EmptyGyro implements IGyro {

	@Override
	public void setYaw(Rotation2d heading) {}

	@Override
	public void updateInputs(GyroInputsAutoLogged inputs) {}

}
