package frc.robot.subsystems.swerve.gyro.gyropigeon2;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;

public class GyroPigeon2Actions {

    private Pigeon2 pigeon2;

    protected GyroPigeon2Actions(Pigeon2 pigeon2) {
        this.pigeon2 = pigeon2;
    }

    public void setYaw(Rotation2d angle) {
        pigeon2.setYaw(angle.getDegrees());
    }
}
