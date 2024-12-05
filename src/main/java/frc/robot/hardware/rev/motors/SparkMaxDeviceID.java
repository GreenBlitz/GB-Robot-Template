package frc.robot.hardware.rev.motors;

import com.revrobotics.CANSparkLowLevel;

public record SparkMaxDeviceID(int id, CANSparkLowLevel.MotorType type) {

	public SparkMaxDeviceID(int id) {
		this(id, CANSparkLowLevel.MotorType.kBrushless);
	}

}
