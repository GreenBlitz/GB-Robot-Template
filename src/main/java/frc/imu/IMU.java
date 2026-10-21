package frc.imu;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.IIMU;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.subsystems.swerve.IMUSignals;

public class IMU {

    private final IIMU imu;
    private final IMUSignals imuSignals;

    public IMU(IIMU imu, IMUSignals imuSignals){
        this.imu = imu;
        this.imuSignals = imuSignals;
    }

    public void setYaw(Rotation2d yaw){
        imu.setYaw(yaw);
    }

    public boolean isConnected(){
        return imu.isConnected();
    }

    public void updateInputs(IMUSignals imuSignals){
        imu.updateInputs(
                imuSignals.pitchSignal(),
                imuSignals.rollSignal(),
                imuSignals.yawSignal(),
                imuSignals.rollAngularVelocitySignal(),
                imuSignals.pitchAngularVelocitySignal(),
                imuSignals.yawAngularVelocitySignal(),
                imuSignals.xAccelerationGSignal(),
                imuSignals.yAccelerationGSignal(),
                imuSignals.zAccelerationGSignal()
        );
    }


}
