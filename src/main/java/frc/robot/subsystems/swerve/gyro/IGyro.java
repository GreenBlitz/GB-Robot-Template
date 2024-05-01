package frc.robot.subsystems.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IGyro {

    void setHeading(Rotation2d heading);

    void updateInputs(GyroInputsAutoLogged inputs);

}
