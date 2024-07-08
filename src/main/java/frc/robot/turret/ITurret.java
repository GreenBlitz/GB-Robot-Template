package frc.robot.turret;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ITurret {
    void setVelocity (double velocity);
    void setPosition (Rotation2d angle);
    void stop();
    void updateInputs(TurretInputsAutoLogged inputs);
}
