package frc.robot.subsystems.solenoid.factory;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.IDs;
import frc.robot.hardware.motor.talonsrx.TalonSRXMotor;
import frc.robot.hardware.signal.supplied.SuppliedDoubleSignal;
import frc.robot.subsystems.solenoid.SolenoidComponents;

public class SolenoidRealConstants {

	protected static SolenoidComponents generateSolenoidComponents(String logPath) {
		TalonSRX motor = new TalonSRX(IDs.TalonSRXs.SOLENOID);
		SuppliedDoubleSignal voltageSignal = new SuppliedDoubleSignal("voltage", motor::getMotorOutputVoltage);
		TalonSRXMotor solenoid = new TalonSRXMotor(logPath, motor, Rotation2d.fromRotations(1));
		return new SolenoidComponents(logPath, solenoid, voltageSignal);
	}

}
