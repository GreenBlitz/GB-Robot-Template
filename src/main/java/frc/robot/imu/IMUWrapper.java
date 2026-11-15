package frc.robot.imu;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.RobotConstants;
import frc.robot.hardware.empties.EmptyIMU;
import frc.robot.hardware.interfaces.IIMU;
import frc.robot.hardware.interfaces.InputSignal;
import frc.utils.TimedValue;
import frc.utils.angle2d.Angle2d;
import org.littletonrobotics.junction.Logger;

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

	public InputSignal<Angle2d> getYawSignal() {
		return imuSignals.yawSignal();
	}

	public Angle2d[] getLatestAngularVelocities() {
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

	public TimedValue<Angle2d> getWrappedYaw() {
		TimedValue<Angle2d> latestYaw = getYawSignal().getLatestTimedValue();
		Angle2d latestAbsoluteYaw = Angle2d.fromRadians(MathUtil.angleModulus(latestYaw.getValue().getRadians()));
		return new TimedValue<>(latestAbsoluteYaw, latestYaw.getTimestamp());
	}


	public void setYaw(Angle2d yaw) {
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
			imuSignals.zAccelerationGSignal()
		);
		Logger.recordOutput(imu.getLogPath() + "Acceleration", getLatestAccelerationMetersPerSecondSquared());
		Logger.recordOutput(imu.getLogPath() + "isConnected", isConnected());
	}

}
