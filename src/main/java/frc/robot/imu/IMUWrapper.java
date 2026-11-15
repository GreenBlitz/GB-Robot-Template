package frc.robot.imu;

import frc.robot.RobotConstants;
import frc.robot.hardware.empties.EmptyIMU;
import frc.robot.hardware.interfaces.IIMU;
import frc.robot.hardware.interfaces.InputSignal;
import frc.utils.TimedValue;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.util.MathUtil;

public class IMUWrapper {

  private final IIMU imu;
  private final IMUSignals imuSignals;

  public IMUWrapper(IIMU imu, IMUSignals imuSignals) {
    this.imu = imu;
    this.imuSignals = imuSignals;
  }

  public boolean isEmpty() {
    return imu instanceof EmptyIMU;
  }

  public boolean isConnected() {
    return imu.isConnected();
  }

  public InputSignal<Rotation2d> getYawSignal() {
    return imuSignals.yawSignal();
  }

  public Rotation2d[] getLatestAngularVelocities() {
    return imuSignals.getLatestAngularVelocity();
  }

  public Rotation3d[] getAllOrientations() {
    return imuSignals.getAllOrientations();
  }

  public Rotation3d getLatestOrientation() {
    return imuSignals.getLatestOrientation();
  }

  public Translation3d[] getAllAccelerationsG() {
    return imuSignals.getAllAccelerationsG();
  }

  public Translation3d getLatestAccelerationG() {
    return imuSignals.getLatestAccelerationG();
  }

  public Translation3d getLatestAccelerationMetersPerSecondSquared() {
    return getLatestAccelerationG().times(RobotConstants.G);
  }

  public TimedValue<Rotation2d> getWrappedYaw() {
    TimedValue<Rotation2d> latestYaw = getYawSignal().getLatestTimedValue();
    Rotation2d latestAbsoluteYaw =
        Rotation2d.fromRadians(MathUtil.angleModulus(latestYaw.getValue().getRadians()));
    return new TimedValue<>(latestAbsoluteYaw, latestYaw.getTimestamp());
  }

  public void setYaw(Rotation2d yaw) {
    imu.setYaw(yaw);
  }

  public void updateIMU() {
    imu.updateInputs(
        imuSignals.pitchSignal(),
        imuSignals.rollSignal(),
        imuSignals.yawSignal(),
        imuSignals.rollAngularVelocitySignal(),
        imuSignals.pitchAngularVelocitySignal(),
        imuSignals.yawAngularVelocitySignal(),
        imuSignals.xAccelerationGSignal(),
        imuSignals.yAccelerationGSignal(),
        imuSignals.zAccelerationGSignal());
    Logger.recordOutput(
        imu.getLogPath() + "Acceleration", getLatestAccelerationMetersPerSecondSquared());
    Logger.recordOutput(imu.getLogPath() + "isConnected", isConnected());
  }
}
