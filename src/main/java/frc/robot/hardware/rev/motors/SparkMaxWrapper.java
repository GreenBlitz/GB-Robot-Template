package frc.robot.hardware.rev.motors;

import com.revrobotics.CANSparkMax;

public class SparkMaxWrapper extends CANSparkMax {

	public SparkMaxWrapper(SparkMaxDeviceID deviceID) {
		super(deviceID.ID(), deviceID.type());
		super.restoreFactoryDefaults();
	}

	public double getVoltage() {
		return getAppliedOutput() * getBusVoltage();
	}

}