package frc.robot.subsystems.swerve.swervegyro.swervegyrointerface;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ISwerveGyro {

    void setHeading(Rotation2d heading);

    void updateInputs(SwerveGyroInputsAutoLogged inputs);

}
