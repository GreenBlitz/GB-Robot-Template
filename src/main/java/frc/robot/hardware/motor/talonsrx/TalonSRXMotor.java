package frc.robot.hardware.motor.talonsrx;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.request.IRequest;
import frc.robot.hardware.request.srx.AngleSRXRequest;
import frc.robot.hardware.request.srx.DoubleSRXRequest;
import frc.robot.hardware.signal.InputSignal;
import frc.utils.Conversions;
import frc.utils.calibration.sysid.SysIdCalibrator;
import org.littletonrobotics.junction.Logger;

public class TalonSRXMotor implements ControllableMotor {

	private final String logPath;
	private final TalonSRX motor;
	private final double gearRatio;

	public TalonSRXMotor(String logPath, TalonSRX talonSRX, double gearRatio) {
		this.logPath = logPath;
		this.motor = talonSRX;
		this.gearRatio = gearRatio;
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
	public SysIdCalibrator.SysIdConfigInfo getSysidConfigInfo() {
		return null;
	}

	@Override
	public void resetPosition(Rotation2d position) {
		motor.setSelectedSensorPosition(Conversions.angleToMagTicks(position, gearRatio));
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

	@Override
	public void applyDoubleRequest(IRequest<Double> request) {
		DoubleSRXRequest srxRequest = (DoubleSRXRequest) request;
		motor.set(srxRequest.getControlMode(), srxRequest.getSetPoint());
	}

	@Override
	public void applyAngleRequest(IRequest<Rotation2d> request) {
		AngleSRXRequest srxRequest = (AngleSRXRequest) request;
		motor.selectProfileSlot(srxRequest.getPidSlot(), 0);
		motor.set(srxRequest.getControlMode(), Conversions.angleToMagTicks(srxRequest.getSetPoint(), gearRatio));
	}

}
