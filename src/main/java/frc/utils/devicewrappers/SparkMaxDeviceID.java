package frc.utils.devicewrappers;

import com.revrobotics.CANSparkLowLevel;

public record SparkMaxDeviceID(int ID, CANSparkLowLevel.MotorType type) {

	public SparkMaxDeviceID(int ID) {
		this(ID, CANSparkLowLevel.MotorType.kBrushless);
	}

}
