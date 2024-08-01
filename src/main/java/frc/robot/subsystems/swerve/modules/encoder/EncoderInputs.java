package frc.robot.subsystems.swerve.modules.encoder;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class EncoderInputs {

    public boolean isConnected = true;
    public Rotation2d angle = new Rotation2d();
    public Rotation2d velocity = new Rotation2d();
    public double voltage = 0;

}
