package frc.robot.subsystems.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ISwerveGyro {

	void setYaw(Rotation2d heading);

	void updateInputs(SwerveGyroInputsAutoLogged inputs);

}
