package frc.robot.hardware.rev.motors;

import com.revrobotics.spark.SparkLowLevel;

public class BrushedSparkMAXMotor extends SparkMaxMotor {

	public BrushedSparkMAXMotor(String logPath, SparkMaxWrapper motor) {
		super(logPath, motor);
		if (motor.getMotorType() != SparkLowLevel.MotorType.kBrushed) {
			throw new IllegalArgumentException("inserted BrushlessSparkMAXMotor to BrushedSparkMAXMotor!");
		}
	}

}
