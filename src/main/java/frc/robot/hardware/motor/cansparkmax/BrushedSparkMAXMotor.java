package frc.robot.hardware.motor.cansparkmax;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

public class BrushedSparkMAXMotor extends SparkMaxMotor {

	protected final CANSparkMax motor;

	public BrushedSparkMAXMotor(CANSparkMax motor, String logPath) {
		super(motor, logPath);
		if (motor.getMotorType() != CANSparkLowLevel.MotorType.kBrushed) {
			throw new IllegalArgumentException("inserted brushless sparkmax motor to brushed!");
		}
		this.motor = motor;
	}

}
