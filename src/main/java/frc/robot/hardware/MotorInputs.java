package frc.robot.hardware;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class MotorInputs {

    public boolean connected = false;
    public Rotation2d angle = new Rotation2d();
    public Rotation2d velocity = new Rotation2d();
    public double current = 0;
    public double voltage = 0;

}
