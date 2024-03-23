package frc.robot.subsystems.swerve.falconswerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.SwerveIO;
import frc.robot.subsystems.swerve.SwerveInputsAutoLogged;

import java.util.Queue;

public class FalconSwerve extends SwerveIO {

    private final Pigeon2 gyro = FalconSwerveConstants.GYRO.get();
    private final Queue<Double>
            yawQueue = TalonFXOdometryThread6328.getInstance().registerSignal(gyro, FalconSwerveConstants.YAW_SIGNAL),
            timestampQueue = TalonFXOdometryThread6328.getInstance().getTimestampQueue();

    @Override
    protected void setHeading(Rotation2d heading) {
        gyro.setYaw(heading.getDegrees());
    }

    private void refreshStatusSignals() {
        BaseStatusSignal.refreshAll(
                FalconSwerveConstants.YAW_SIGNAL,
                FalconSwerveConstants.PITCH_SIGNAL,
                FalconSwerveConstants.X_ACCELERATION_SIGNAL,
                FalconSwerveConstants.Y_ACCELERATION_SIGNAL,
                FalconSwerveConstants.Z_ACCELERATION_SIGNAL
        );
    }

    @Override
    protected void updateInputs(SwerveInputsAutoLogged inputs) {
        refreshStatusSignals();

        inputs.gyroYawDegrees = FalconSwerveConstants.YAW_SIGNAL.getValue();
        inputs.gyroPitchDegrees = FalconSwerveConstants.PITCH_SIGNAL.getValue();
        inputs.accelerationX = FalconSwerveConstants.X_ACCELERATION_SIGNAL.getValue();
        inputs.accelerationY = FalconSwerveConstants.Y_ACCELERATION_SIGNAL.getValue();
        inputs.accelerationZ = FalconSwerveConstants.Z_ACCELERATION_SIGNAL.getValue();
        inputs.odometryUpdatesYawDegrees = yawQueue.stream().mapToDouble(Double::doubleValue).toArray();

        inputs.odometryUpdatesTimestamp = timestampQueue.stream().mapToDouble(Double::doubleValue).toArray();

        yawQueue.clear();
        timestampQueue.clear();
    }

}
