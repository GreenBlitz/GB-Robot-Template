package frc.robot.hardware.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class GyroInputs {

    public boolean connected = false;
    public Rotation2d yaw = new Rotation2d();

}


