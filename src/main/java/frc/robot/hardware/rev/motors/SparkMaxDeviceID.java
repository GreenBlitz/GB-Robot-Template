package frc.robot.hardware.rev.motors;

import com.revrobotics.spark.SparkLowLevel;

public record SparkMaxDeviceID(int id, SparkLowLevel.MotorType type) {

	public SparkMaxDeviceID(int id) {
		this(id, SparkLowLevel.MotorType.kBrushless);
	}

}
