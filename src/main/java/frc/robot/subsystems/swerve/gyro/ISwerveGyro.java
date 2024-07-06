package frc.robot.subsystems.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ISwerveGyro {

    void setHeading(Rotation2d heading);

    void updateInputs(SwerveGyroInputsAutoLogged inputs);

}
