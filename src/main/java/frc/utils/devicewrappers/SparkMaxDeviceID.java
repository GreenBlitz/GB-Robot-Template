package frc.utils.devicewrappers;

import com.revrobotics.CANSparkLowLevel;

public class SparkMaxDeviceID {

	private int id;
	private CANSparkLowLevel.MotorType type;

	public SparkMaxDeviceID(int id, CANSparkLowLevel.MotorType type) {
		this.id = id;
		this.type = type;
	}

	public int getId() {
		return id;
	}

	public CANSparkLowLevel.MotorType getType() {
		return type;
	}

	public SparkMaxDeviceID withID(int id) {
		this.id = id;
		return this;
	}

	public SparkMaxDeviceID withType(CANSparkLowLevel.MotorType type) {
		this.type = type;
		return this;
	}

}
