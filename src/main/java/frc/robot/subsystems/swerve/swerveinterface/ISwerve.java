package frc.robot.subsystems.swerve.swerveinterface;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ISwerve {

    void setHeading(Rotation2d heading);

    void updateInputs(SwerveInputsAutoLogged inputs);
}
