package frc.robot.subsystems.swerve.swervegyro.pigeon2swervegyro;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.Phoenix6Constants;
import frc.robot.constants.Ports;
import frc.robot.subsystems.swerve.PhoenixOdometryThread6328;
import frc.robot.subsystems.swerve.swervegyro.swervegyrointerface.ISwerveGyro;
import frc.robot.subsystems.swerve.swervegyro.swervegyrointerface.SwerveGyroInputsAutoLogged;

import java.util.Queue;

public class Pigeon2SwerveGyro implements ISwerveGyro {

    private final Pigeon2SwerveGyroStatus gyroPigeon2Status;

    private final Pigeon2SwerveGyroActions gyroPigeon2Actions;

    private final Queue<Double> yawQueue;

    private final Queue<Double> timestampQueue;

    public Pigeon2SwerveGyro() {
        Pigeon2SwerveGyroConfigObject gyroPigeon2ConfigObject = new Pigeon2SwerveGyroConfigObject(
                Ports.PIGEON_2_ID,
                Phoenix6Constants.CANIVORE_NAME
        );

        this.gyroPigeon2Status = new Pigeon2SwerveGyroStatus(gyroPigeon2ConfigObject.getSignals());
        this.gyroPigeon2Actions = new Pigeon2SwerveGyroActions(gyroPigeon2ConfigObject.getGyro());

        this.yawQueue = PhoenixOdometryThread6328.getInstance().registerRegularSignal(
                gyroPigeon2ConfigObject.getGyro(),
                gyroPigeon2Status.getYAW_SIGNAL(false)
        );
        this.timestampQueue = PhoenixOdometryThread6328.getInstance().getTimestampQueue();
    }

    @Override
    public void setHeading(Rotation2d heading) {
        gyroPigeon2Actions.setYaw(heading);
    }

    @Override
    public void updateInputs(SwerveGyroInputsAutoLogged inputs) {
        gyroPigeon2Status.refreshAllSignals();

        inputs.gyroYaw = gyroPigeon2Status.getYaw(false);
        inputs.gyroPitch = gyroPigeon2Status.getPitch(false);
        inputs.accelerationX = gyroPigeon2Status.getX_ACCELERATION_SIGNAL(false).getValue();
        inputs.accelerationY = gyroPigeon2Status.getY_ACCELERATION_SIGNAL(false).getValue();
        inputs.accelerationZ = gyroPigeon2Status.getZ_ACCELERATION_SIGNAL(false).getValue();

        inputs.odometryUpdatesYaw = yawQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);
        inputs.odometryUpdatesTimestamp = timestampQueue.stream().mapToDouble(Double::doubleValue).toArray();

        yawQueue.clear();
        timestampQueue.clear();
    }

}
