package frc.robot.hardware.motor.sparkmax;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import frc.robot.hardware.ConnectedInputAutoLogged;
import frc.robot.hardware.motor.IMotor;
import frc.robot.hardware.signal.InputSignal;
import frc.robot.hardware.signal.cansparkmax.ISparkMaxSignal;
import frc.utils.alerts.Alert;
import frc.utils.alerts.AlertManager;
import frc.utils.alerts.PeriodicAlert;
import org.littletonrobotics.junction.Logger;

public abstract class SparkMaxMotor implements IMotor {

	protected final CANSparkMax motor;
	private final String logPath;
	private final ConnectedInputAutoLogged connectedInput;

	public SparkMaxMotor(String logPath, CANSparkMax motor) {
		this.logPath = logPath;
		this.motor = motor;

		this.connectedInput = new ConnectedInputAutoLogged();
		connectedInput.connected = true;

		AlertManager.addAlert(new PeriodicAlert(Alert.AlertType.ERROR, logPath + "disconnectedAt", () -> !isConnected()));
	}
	
	@Override
	public boolean isConnected() {
		return connectedInput.connected;
	}
	
	@Override
	public void updateSignals(InputSignal... signals) {
		for (InputSignal signal : signals) {
			if (signal instanceof ISparkMaxSignal) {
				Logger.processInputs(logPath, signal);
			}
		}
	}

	@Override
	public void setBrake(boolean brake) {
		CANSparkBase.IdleMode idleMode = brake ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast;
		motor.setIdleMode(idleMode);
	}

	@Override
	public void stop() {
		motor.stopMotor();
	}

	@Override
	public void setPower(double power) {
		motor.set(power);
	}

}
