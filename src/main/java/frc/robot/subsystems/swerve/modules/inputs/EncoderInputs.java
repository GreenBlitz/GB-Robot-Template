package frc.robot.subsystems.swerve.modules.inputs;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class EncoderInputs {

    public boolean isEncoderConnected = true;// todo - test for each component
    public Rotation2d encoderAngle = new Rotation2d();
    public Rotation2d encoderVelocity = new Rotation2d();
    public double encoderVoltage = 0;

}
