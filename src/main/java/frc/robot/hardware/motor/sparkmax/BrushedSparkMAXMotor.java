package frc.robot.hardware.motor.sparkmax;

import com.revrobotics.CANSparkLowLevel;

public class BrushedSparkMAXMotor extends SparkMaxMotor {

	public BrushedSparkMAXMotor(String logPath, SparkMaxWrapper motor) {
		super(logPath, motor);
		if (motor.getMotorType() != CANSparkLowLevel.MotorType.kBrushed) {
			throw new IllegalArgumentException("inserted BrushlessSparkMAXMotor to BrushedSparkMAXMotor!");
		}
	}

}
