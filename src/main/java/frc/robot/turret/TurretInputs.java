package frc.robot.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class TurretInputs {

    public Rotation2d position;
    public Rotation2d targetPosition;
    public Rotation2d velocity;
    public Rotation2d targetVelocity;
}
