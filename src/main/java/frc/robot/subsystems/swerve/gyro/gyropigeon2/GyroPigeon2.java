package frc.robot.subsystems.swerve.gyro.gyropigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.Phoenix6Constants;
import frc.robot.constants.Ports;
import frc.robot.subsystems.swerve.gyro.GyroInputsAutoLogged;
import frc.robot.subsystems.swerve.gyro.IGyro;

public class GyroPigeon2 implements IGyro {

    private final GyroPigeon2Status gyroPigeon2Status;

    private final GyroPigeon2Actions gyroPigeon2Actions;

    public GyroPigeon2() {
        final GyroPigeon2ConfigObject gyroPigeon2ConfigObject = new GyroPigeon2ConfigObject(
                Ports.PIGEON_2_ID,
                Phoenix6Constants.CANIVORE_NAME
        );

        this.gyroPigeon2Status = new GyroPigeon2Status(gyroPigeon2ConfigObject.getSignals());
        this.gyroPigeon2Actions = new GyroPigeon2Actions(gyroPigeon2ConfigObject.getGyro());
    }

    @Override
    public void setHeading(Rotation2d heading) {
        gyroPigeon2Actions.setYaw(heading);
    }

    @Override
    public void updateInputs(GyroInputsAutoLogged inputs) {
        inputs.gyroYawDegrees = gyroPigeon2Status.getYAW_SIGNAL().getValue();
        inputs.gyroPitchDegrees = gyroPigeon2Status.getPITCH_SIGNAL().getValue();
        inputs.accelerationX = gyroPigeon2Status.getX_ACCELERATION_SIGNAL().getValue();
        inputs.accelerationY = gyroPigeon2Status.getY_ACCELERATION_SIGNAL().getValue();
        inputs.accelerationZ = gyroPigeon2Status.getZ_ACCELERATION_SIGNAL().getValue();
    }

}
