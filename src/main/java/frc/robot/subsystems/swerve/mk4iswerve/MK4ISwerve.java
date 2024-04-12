package frc.robot.subsystems.swerve.mk4iswerve;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.gyro.gyropigeon2.GyroPigeon2;
import frc.robot.subsystems.swerve.swerveinterface.ISwerve;
import frc.robot.subsystems.swerve.swerveinterface.SwerveInputsAutoLogged;

public class MK4ISwerve implements ISwerve {

    private final GyroPigeon2 pigeon2Gyro;

    public MK4ISwerve() {
        this.pigeon2Gyro = new GyroPigeon2();
    }

    @Override
    public void setHeading(Rotation2d heading) {
        pigeon2Gyro.setYaw(heading);
    }

    @Override
    public void updateInputs(SwerveInputsAutoLogged inputs) {
        inputs.gyroYawDegrees = pigeon2Gyro.getYAW_SIGNAL().getValue();
        inputs.gyroPitchDegrees = pigeon2Gyro.getPITCH_SIGNAL().getValue();
        inputs.accelerationX = pigeon2Gyro.getX_ACCELERATION_SIGNAL().getValue();
        inputs.accelerationY = pigeon2Gyro.getY_ACCELERATION_SIGNAL().getValue();
        inputs.accelerationZ = pigeon2Gyro.getZ_ACCELERATION_SIGNAL().getValue();
    }

}
