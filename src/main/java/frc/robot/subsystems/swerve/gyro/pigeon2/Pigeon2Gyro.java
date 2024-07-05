package frc.robot.subsystems.swerve.gyro.pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.Ports;
import frc.robot.subsystems.swerve.gyro.SwerveGyroConstants;
import frc.robot.subsystems.swerve.gyro.gyrointerface.ISwerveGyro;
import frc.robot.subsystems.swerve.gyro.gyrointerface.SwerveGyroInputsAutoLogged;
import frc.robot.subsystems.swerve.odometryThread.PhoenixOdometryThread6328;
import org.littletonrobotics.junction.Logger;

import java.util.Queue;

public class Pigeon2Gyro implements ISwerveGyro {

    private final Pigeon2GyroStatus gyroPigeon2Status;
    private final Pigeon2GyroActions gyroPigeon2Actions;

    private final Queue<Double> yawQueue;
    private final Queue<Double> timestampQueue;

    public Pigeon2Gyro() {
        Pigeon2GyroConfigObject gyroPigeon2ConfigObject = new Pigeon2GyroConfigObject(Ports.PIGEON_2_DEVICE_ID);

        this.gyroPigeon2Status = new Pigeon2GyroStatus(gyroPigeon2ConfigObject.getSignals());
        this.gyroPigeon2Actions = new Pigeon2GyroActions(gyroPigeon2ConfigObject.getGyro());

        this.yawQueue = PhoenixOdometryThread6328.getInstance().registerRegularSignal(
                gyroPigeon2ConfigObject.getGyro(),
                gyroPigeon2Status.getYawSignal(false)
        );//todo - maybe latency
        this.timestampQueue = PhoenixOdometryThread6328.getInstance().getTimestampQueue();
    }

    @Override
    public void setHeading(Rotation2d heading) {
        gyroPigeon2Actions.setYaw(heading);
    }


    private void reportAlerts(SwerveGyroInputsAutoLogged inputs) {
        if (!inputs.connected) {
            Logger.recordOutput(SwerveGyroConstants.ALERT_LOG_PATH + "/gyroDisconnectedAt", Timer.getFPGATimestamp());
        }
    }

    @Override
    public void updateInputs(SwerveGyroInputsAutoLogged inputs) {
        inputs.connected = gyroPigeon2Status.refreshAllSignals().isOK();

        inputs.gyroYaw = gyroPigeon2Status.getYaw(false);
        inputs.accelerationX = gyroPigeon2Status.getXAccelerationSignal(false).getValue();
        inputs.accelerationY = gyroPigeon2Status.getYAccelerationSignal(false).getValue();
        inputs.accelerationZ = gyroPigeon2Status.getZAccelerationSignal(false).getValue();

        inputs.odometrySamplesYaw = yawQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);
        inputs.odometrySamplesTimestamp = timestampQueue.stream().mapToDouble(Double::doubleValue).toArray();

        yawQueue.clear();
        timestampQueue.clear();

        reportAlerts(inputs);
    }


}
