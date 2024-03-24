package frc.robot.subsystems.swerve.mk4iswerve;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Gyro.pigeon2.Pigeon2GyroConfigObject;
import frc.robot.subsystems.swerve.swerveinterface.ISwerve;
import frc.robot.subsystems.swerve.swerveinterface.SwerveInputsAutoLogged;

import java.util.Queue;

public class MK4ISwerve implements ISwerve {

    private final Pigeon2GyroConfigObject pigeon2GyroConfigObject;
    private final Queue<Double> yawQueue, timestampQueue;


    public MK4ISwerve(){
        pigeon2GyroConfigObject = MK4ISwerveConstants.PIGEON_2_GYRO_CONFIG_OBJECT;
        yawQueue = TalonFXOdometryThread6328.getInstance().registerSignal(pigeon2GyroConfigObject.getGyro(), pigeon2GyroConfigObject.YAW_SIGNAL);
        timestampQueue = TalonFXOdometryThread6328.getInstance().getTimestampQueue();
    }

    @Override
    public void setHeading(Rotation2d heading) {
        pigeon2GyroConfigObject.getGyro().setYaw(heading.getDegrees());
    }


    private void refreshStatusSignals() {
        BaseStatusSignal.refreshAll(
                pigeon2GyroConfigObject.YAW_SIGNAL,
                pigeon2GyroConfigObject.PITCH_SIGNAL,
                pigeon2GyroConfigObject.X_ACCELERATION_SIGNAL,
                pigeon2GyroConfigObject.Y_ACCELERATION_SIGNAL,
                pigeon2GyroConfigObject.Z_ACCELERATION_SIGNAL
        );
    }

    @Override
    public void updateInputs(SwerveInputsAutoLogged inputs) {
        refreshStatusSignals();

        inputs.gyroYawDegrees = pigeon2GyroConfigObject.YAW_SIGNAL.getValue();
        inputs.gyroPitchDegrees = pigeon2GyroConfigObject.PITCH_SIGNAL.getValue();
        inputs.accelerationX = pigeon2GyroConfigObject.X_ACCELERATION_SIGNAL.getValue();
        inputs.accelerationY = pigeon2GyroConfigObject.Y_ACCELERATION_SIGNAL.getValue();
        inputs.accelerationZ = pigeon2GyroConfigObject.Z_ACCELERATION_SIGNAL.getValue();
        inputs.odometryUpdatesYawDegrees = yawQueue.stream().mapToDouble(Double::doubleValue).toArray();

        inputs.odometryUpdatesTimestamp = timestampQueue.stream().mapToDouble(Double::doubleValue).toArray();

        yawQueue.clear();
        timestampQueue.clear();
    }


}
