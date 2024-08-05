package frc.robot.subsystems.swerve.gyro.pigeon2;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.GlobalConstants;
import frc.robot.poseestimation.PoseEstimatorConstants;
import frc.robot.subsystems.swerve.gyro.ISwerveGyro;
import frc.robot.subsystems.swerve.gyro.SwerveGyroConstants;
import frc.robot.subsystems.swerve.gyro.SwerveGyroInputsAutoLogged;
import frc.robot.subsystems.swerve.odometryThread.PhoenixOdometryThread6328;
import frc.utils.ctre.CTREDeviceID;
import frc.utils.devicewrappers.Pigeon2Wrapper;
import org.littletonrobotics.junction.Logger;

import java.util.Queue;

public class Pigeon2Gyro implements ISwerveGyro {

    private final Pigeon2 gyro;
    private final StatusSignal<Double> yawSignal, xAccelerationSignal, yAccelerationSignal, zAccelerationSignal;
    private final Queue<Double> yawQueue, timestampQueue;
    private final SwerveGyroConstants constants;

    public Pigeon2Gyro(CTREDeviceID gyroID, Pigeon2Configuration configuration, String swerveLogPath){
        this.gyro = new Pigeon2Wrapper(gyroID);
        this.constants = new SwerveGyroConstants(swerveLogPath);

        this.yawSignal = gyro.getYaw().clone();
        this.xAccelerationSignal = gyro.getAccelerationX().clone();
        this.yAccelerationSignal = gyro.getAccelerationY().clone();
        this.zAccelerationSignal = gyro.getAccelerationZ().clone();

        //todo - maybe latency
        this.yawQueue = PhoenixOdometryThread6328.getInstance().registerRegularSignal(gyro, yawSignal);
        this.timestampQueue = PhoenixOdometryThread6328.getInstance().getTimestampQueue();

        configGyro(configuration);
        optimizeBusAndSignals();
    }

    private void configGyro(Pigeon2Configuration configuration) {
        gyro.getConfigurator().apply(configuration);
    }

    private void optimizeBusAndSignals() {
        BaseStatusSignal.setUpdateFrequencyForAll(
                PoseEstimatorConstants.ODOMETRY_FREQUENCY_HERTZ,
                yawSignal
        );
        BaseStatusSignal.setUpdateFrequencyForAll(
                GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
                xAccelerationSignal,
                yAccelerationSignal,
                zAccelerationSignal
        );

        gyro.optimizeBusUtilization();
    }


    @Override
    public void setYaw(Rotation2d heading) {
        gyro.setYaw(heading.getDegrees());
    }


    private void reportAlerts(SwerveGyroInputsAutoLogged inputs) {
        if (!inputs.isConnected) {
            Logger.recordOutput(constants.alertLogPath() + "gyroDisconnectedAt", Timer.getFPGATimestamp());
        }
    }

    @Override
    public void updateInputs(SwerveGyroInputsAutoLogged inputs) {
        inputs.isConnected = BaseStatusSignal.refreshAll(
                yawSignal,
                xAccelerationSignal,
                yAccelerationSignal,
                zAccelerationSignal
        ).isOK();
        inputs.gyroYaw = Rotation2d.fromDegrees(yawSignal.getValue());
        inputs.xAcceleration = xAccelerationSignal.getValue();
        inputs.yAcceleration = yAccelerationSignal.getValue();
        inputs.zAcceleration = zAccelerationSignal.getValue();
        inputs.yawOdometrySamples = yawQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);
        inputs.timestampOdometrySamples = timestampQueue.stream().mapToDouble(Double::doubleValue).toArray();
        yawQueue.clear();
        timestampQueue.clear();

        reportAlerts(inputs);
    }

}
