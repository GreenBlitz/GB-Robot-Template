package frc.robot.subsystems.swerve.swervegyro.pigeon2swervegyro;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.utils.devicewrappers.Pigeon2Wrapper;

class Pigeon2SwerveGyroActions {

    private final Pigeon2Wrapper pigeon2;

    protected Pigeon2SwerveGyroActions(Pigeon2Wrapper pigeon2) {
        this.pigeon2 = pigeon2;
    }

    public void setYaw(Rotation2d angle) {
        pigeon2.setYaw(angle.getDegrees());
    }

}
