package frc.robot.hardware.rev.motors;

import com.revrobotics.spark.SparkLowLevel;

public record SparkMaxDeviceID(int busID, int id, SparkLowLevel.MotorType type) {

	public SparkMaxDeviceID(int busID, int id) {
		this(busID, id, SparkLowLevel.MotorType.kBrushless);
	}

}
