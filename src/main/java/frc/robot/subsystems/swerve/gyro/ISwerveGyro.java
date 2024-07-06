package frc.robot.subsystems.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.gyro.gyrointerface.SwerveGyroInputsAutoLogged;

public interface ISwerveGyro {

    void setHeading(Rotation2d heading);

    void updateInputs(SwerveGyroInputsAutoLogged inputs);

}
