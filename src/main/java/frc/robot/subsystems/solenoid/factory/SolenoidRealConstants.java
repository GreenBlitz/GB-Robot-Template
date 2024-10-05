package frc.robot.subsystems.solenoid.factory;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.constants.IDs;
import frc.robot.hardware.motor.talonsrx.TalonSRXMotor;
import frc.robot.hardware.signal.supplied.SuppliedDoubleSignal;
import frc.robot.subsystems.solenoid.SolenoidComponents;

public class SolenoidRealConstants {

	private static final double GEAR_RATIO = 1;

	protected static SolenoidComponents generateSolenoidComponents(String logPath) {
		TalonSRX motor = new TalonSRX(IDs.TalonSRXs.SOLENOID);
		SuppliedDoubleSignal voltageSignal = new SuppliedDoubleSignal("voltage", motor::getMotorOutputVoltage);
		TalonSRXMotor solenoid = new TalonSRXMotor(logPath, motor, GEAR_RATIO);
		return new SolenoidComponents(logPath, solenoid, voltageSignal);
	}

}
