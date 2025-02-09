package frc.robot.subsystems.solenoid.factory;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.hardware.rev.motors.SparkMaxMotor;
import frc.robot.hardware.rev.motors.SparkMaxWrapper;
import frc.robot.hardware.signal.supplied.SuppliedDoubleSignal;
import frc.robot.subsystems.solenoid.SolenoidComponents;
import frc.robot.IDs;

public class SolenoidRealConstants {

	private static final double GEAR_RATIO = 1;

	protected static SolenoidComponents generateSolenoidComponents(String logPath) {
		SparkMaxWrapper motor = new SparkMaxWrapper(IDs.SparkMAXIDs.SOLENOID);
		SuppliedDoubleSignal voltageSignal = new SuppliedDoubleSignal("voltage", motor::getMotorOutputVoltage);
		TalonSRXMotor solenoid = new TalonSRXMotor(logPath, motor, GEAR_RATIO);
		return new SolenoidComponents(logPath, solenoid, voltageSignal);
	}

}
