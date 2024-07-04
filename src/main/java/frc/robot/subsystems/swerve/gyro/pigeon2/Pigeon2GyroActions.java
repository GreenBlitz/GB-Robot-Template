package frc.robot.subsystems.swerve.gyro.pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.utils.devicewrappers.Pigeon2Wrapper;

class Pigeon2GyroActions {

    private final Pigeon2Wrapper pigeon2;

    protected Pigeon2GyroActions(Pigeon2Wrapper pigeon2) {
        this.pigeon2 = pigeon2;
    }

    public void setYaw(Rotation2d angle) {
        pigeon2.setYaw(angle.getDegrees());
    }

}
