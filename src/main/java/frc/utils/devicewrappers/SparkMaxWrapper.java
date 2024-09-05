package frc.utils.devicewrappers;

import com.revrobotics.CANSparkMax;

public class SparkMaxWrapper extends CANSparkMax {

	public SparkMaxWrapper(SparkMaxDeviceID deviceID) {
		super(deviceID.id(), deviceID.type());
		super.restoreFactoryDefaults();
	}

}
