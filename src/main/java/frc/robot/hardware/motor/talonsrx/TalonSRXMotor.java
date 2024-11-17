package frc.robot.hardware.motor.talonsrx;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.hardware.request.srx.AngleSRXRequest;
import frc.robot.hardware.request.srx.DoubleSRXRequest;
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
	public void updateSimulation() {

	}

	@Override
	public void updateInputs(InputSignal<?>... inputSignals) {
		for (InputSignal<?> signal : inputSignals) {
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
	public void applyRequest(IRequest<?> request) {
		if (request instanceof AngleSRXRequest angleSRXRequest) {
			motor.selectProfileSlot(angleSRXRequest.getPidSlot(), 0);
			motor.set(angleSRXRequest.getControlMode(), Conversions.angleToMagTicks(angleSRXRequest.getSetPoint(), gearRatio));
		}
		else if (request instanceof DoubleSRXRequest doubleSRXRequest) {
			motor.set(doubleSRXRequest.getControlMode(), doubleSRXRequest.getSetPoint());
		}
	}

}
