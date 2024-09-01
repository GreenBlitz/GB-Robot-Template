package frc.robot.hardware.motor;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class PIDAbleInputs {

    public Rotation2d angle = new Rotation2d();
    public Rotation2d velocity = new Rotation2d();
    public Rotation2d acceleration = new Rotation2d();

}
