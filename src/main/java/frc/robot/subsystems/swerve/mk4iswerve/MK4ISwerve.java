package frc.robot.subsystems.swerve.mk4iswerve;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.gyro.GyroPigeon2;
import frc.robot.subsystems.swerve.swerveinterface.ISwerve;
import frc.robot.subsystems.swerve.swerveinterface.SwerveInputsAutoLogged;

import java.util.Queue;

public class MK4ISwerve implements ISwerve {

    private final GyroPigeon2 pigeon2Gyro;
    private final Queue<Double> yawQueue, timestampQueue;


    public MK4ISwerve(){
        pigeon2Gyro = GyroPigeon2.getInstance();
        yawQueue = TalonFXOdometryThread6328.getInstance().registerSignal(pigeon2Gyro.getGyro(), pigeon2Gyro.YAW_SIGNAL);
        timestampQueue = TalonFXOdometryThread6328.getInstance().getTimestampQueue();
    }

    @Override
    public void setHeading(Rotation2d heading) {
        pigeon2Gyro.getGyro().setYaw(heading.getDegrees());
    }


    private void refreshStatusSignals() {
        BaseStatusSignal.refreshAll(
                pigeon2Gyro.YAW_SIGNAL,
                pigeon2Gyro.PITCH_SIGNAL,
                pigeon2Gyro.X_ACCELERATION_SIGNAL,
                pigeon2Gyro.Y_ACCELERATION_SIGNAL,
                pigeon2Gyro.Z_ACCELERATION_SIGNAL
        );
    }

    @Override
    public void updateInputs(SwerveInputsAutoLogged inputs) {
        refreshStatusSignals();

        inputs.gyroYawDegrees = pigeon2Gyro.YAW_SIGNAL.getValue();
        inputs.gyroPitchDegrees = pigeon2Gyro.PITCH_SIGNAL.getValue();
        inputs.accelerationX = pigeon2Gyro.X_ACCELERATION_SIGNAL.getValue();
        inputs.accelerationY = pigeon2Gyro.Y_ACCELERATION_SIGNAL.getValue();
        inputs.accelerationZ = pigeon2Gyro.Z_ACCELERATION_SIGNAL.getValue();
        inputs.odometryUpdatesYawDegrees = yawQueue.stream().mapToDouble(Double::doubleValue).toArray();

        inputs.odometryUpdatesTimestamp = timestampQueue.stream().mapToDouble(Double::doubleValue).toArray();

        yawQueue.clear();
        timestampQueue.clear();
    }


}
