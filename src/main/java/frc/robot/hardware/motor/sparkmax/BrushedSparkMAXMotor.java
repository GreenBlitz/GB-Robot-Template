package frc.robot.hardware.motor.sparkmax;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

public class BrushedSparkMAXMotor extends SparkMaxMotor {

	private final CANSparkMax motor;

	public BrushedSparkMAXMotor(String logPath, CANSparkMax motor) {
		super(logPath, motor);
		if (motor.getMotorType() != CANSparkLowLevel.MotorType.kBrushed) {
			throw new IllegalArgumentException("\"inserted BrushlessSparkMAXMotor to BrushedSparkMAXMotor!\"");
		}
		this.motor = motor;
	}

}
