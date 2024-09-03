package frc.robot.hardware.gyro.pigeon2;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.gyro.GyroInputsAutoLogged;
import frc.robot.hardware.gyro.IGyro;
import frc.utils.devicewrappers.Pigeon2Wrapper;

public class Pigeon2Gyro implements IGyro {

	protected final Pigeon2 gyro;
	protected final StatusSignal<Double> yawSignal;

	public Pigeon2Gyro(Pigeon2Wrapper gyro, StatusSignal<Double> yawSignal) {
		this.gyro = gyro;
		this.yawSignal = yawSignal;
	}

	@Override
	public void setYaw(Rotation2d heading) {
		gyro.setYaw(heading.getDegrees());
	}

	@Override
	public void updateInputs(GyroInputsAutoLogged inputs) {
		inputs.connected = BaseStatusSignal.refreshAll(yawSignal).isOK();
		inputs.yaw = Rotation2d.fromDegrees(yawSignal.getValue());
	}

}
