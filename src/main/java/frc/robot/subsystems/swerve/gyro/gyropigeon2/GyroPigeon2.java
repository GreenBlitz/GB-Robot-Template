package frc.robot.subsystems.swerve.gyro.gyropigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.Phoenix6Constants;
import frc.robot.constants.Ports;
import frc.robot.subsystems.swerve.gyro.GyroInputsAutoLogged;
import frc.robot.subsystems.swerve.gyro.IGyro;

import java.util.Queue;

public class GyroPigeon2 implements IGyro {

    private final GyroPigeon2Status gyroPigeon2Status;

    private final GyroPigeon2Actions gyroPigeon2Actions;

    private final Queue<Double> yawQueue;

    private final Queue<Double> timestampQueue;

    public GyroPigeon2() {
        GyroPigeon2ConfigObject gyroPigeon2ConfigObject = new GyroPigeon2ConfigObject(
                Ports.PIGEON_2_ID,
                Phoenix6Constants.CANIVORE_NAME
        );

        this.gyroPigeon2Status = new GyroPigeon2Status(gyroPigeon2ConfigObject.getSignals());
        this.gyroPigeon2Actions = new GyroPigeon2Actions(gyroPigeon2ConfigObject.getGyro());

        this.yawQueue = TalonFXOdometryThread6328.getInstance().registerSignal(
                gyroPigeon2ConfigObject.getGyro(),
                gyroPigeon2Status.getYAW_SIGNAL(false)
        );
        this.timestampQueue = TalonFXOdometryThread6328.getInstance().getTimestampQueue();
    }

    @Override
    public void setHeading(Rotation2d heading) {
        gyroPigeon2Actions.setYaw(heading);
    }

    @Override
    public void updateInputs(GyroInputsAutoLogged inputs) {
        gyroPigeon2Status.refreshAllSignals();

        inputs.gyroYawDegrees = gyroPigeon2Status.getYAW_SIGNAL(false).getValue();
        inputs.gyroPitchDegrees = gyroPigeon2Status.getPITCH_SIGNAL(false).getValue();
        inputs.accelerationX = gyroPigeon2Status.getX_ACCELERATION_SIGNAL(false).getValue();
        inputs.accelerationY = gyroPigeon2Status.getY_ACCELERATION_SIGNAL(false).getValue();
        inputs.accelerationZ = gyroPigeon2Status.getZ_ACCELERATION_SIGNAL(false).getValue();

        inputs.odometryUpdatesYawDegrees = yawQueue.stream().mapToDouble(Double::doubleValue).toArray();
        inputs.odometryUpdatesTimestamp = timestampQueue.stream().mapToDouble(Double::doubleValue).toArray();

        yawQueue.clear();
        timestampQueue.clear();
    }

}
