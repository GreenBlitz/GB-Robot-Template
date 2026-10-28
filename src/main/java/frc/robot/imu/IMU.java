package frc.robot.imu;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.RobotConstants;
import frc.robot.hardware.empties.EmptyIMU;
import frc.robot.hardware.interfaces.IIMU;
import frc.robot.hardware.interfaces.InputSignal;
import frc.utils.TimedValue;
import org.littletonrobotics.junction.Logger;

public class IMU {

	private final IIMU imu;
	private final IMUSignals imuSignals;

	public IMU(IIMU imu, IMUSignals imuSignals) {
		this.imu = imu;
		this.imuSignals = imuSignals;
	}

	public boolean isEmpty() {
		return imu instanceof EmptyIMU;
	}

	public void setYaw(Rotation2d yaw) {
		imu.setYaw(yaw);
	}

	public InputSignal<Rotation2d> getYawSignal() {
		return imuSignals.yawSignal();
	}

	public boolean isConnected() {
		return imu.isConnected();
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
		return imuSignals.getLatestAccelerationGMetersPerSecondSquare();
	}

	public Translation3d getAccelerationMetersPerSecondSquared() {
		return getLatestAccelerationG().times(RobotConstants.G);
	}

	public TimedValue<Rotation2d> getAbsoluteYaw() {
		TimedValue<Rotation2d> latestYaw = imuSignals.yawSignal().getLatestTimedValue();
		Rotation2d latestAbsoluteYaw = Rotation2d.fromRadians(MathUtil.angleModulus(latestYaw.getValue().getRadians()));
		return new TimedValue<>(latestAbsoluteYaw, latestYaw.getTimestamp());
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
			imuSignals.zAccelerationGSignal()
		);
		Logger.recordOutput("IMU/Acceleration", getAccelerationMetersPerSecondSquared());
	}

}
