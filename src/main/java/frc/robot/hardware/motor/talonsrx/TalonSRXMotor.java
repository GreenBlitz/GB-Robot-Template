package frc.robot.hardware.motor.talonsrx;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.hardware.motor.IMotor;
import frc.robot.hardware.signal.InputSignal;
import org.littletonrobotics.junction.Logger;

public class TalonSRXMotor implements IMotor {

	private final String logPath;
	private final TalonSRX motor;

	public TalonSRXMotor(String logPath, TalonSRX talonSRX) {
		this.logPath = logPath;
		this.motor = talonSRX;
	}

	@Override
	public boolean isConnected() {
		return true; // TODO
	}

	@Override
	public void updateSignals(InputSignal... signals) {
		for (InputSignal signal : signals) {
			Logger.processInputs(logPath, signal);
		}
	}

	@Override
	public void setBrake(boolean brake) {
		NeutralMode neutralMode = brake ? NeutralMode.Brake : NeutralMode.Coast;
		motor.setNeutralMode(neutralMode);
	}

	@Override
	public void stop() {
		motor.set(ControlMode.PercentOutput, 0);
	}

	@Override
	public void setPower(double power) {
		motor.set(TalonSRXControlMode.PercentOutput, power);
	}

}
