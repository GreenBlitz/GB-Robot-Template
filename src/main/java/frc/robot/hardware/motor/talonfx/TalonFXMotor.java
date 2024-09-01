package frc.robot.hardware.motor.talonfx;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hardware.motor.*;
import frc.robot.hardware.motor.IMotor;
import frc.robot.hardware.motor.PIDAble;
import frc.robot.hardware.request.IControlRequest;
import frc.robot.hardware.request.TalonFXControlRequest;
import frc.utils.calibration.sysid.SysIdCalibrator;
import frc.utils.devicewrappers.TalonFXWrapper;

public class TalonFXMotor implements IMotor, PIDAble, ProfileAble {

	protected final TalonFXWrapper motor;
	protected final TalonFXSignals signals;
	protected final SysIdCalibrator.SysIdConfigInfo sysidConfigInfo;

	public TalonFXMotor(TalonFXWrapper motor, TalonFXSignals signals, SysIdRoutine.Config sysidConfig) {
		this.motor = motor;
		this.signals = signals;
		this.sysidConfigInfo = new SysIdCalibrator.SysIdConfigInfo(sysidConfig, true);
	}

	@Override
	public SysIdCalibrator.SysIdConfigInfo getSysidConfigInfo() {
		return sysidConfigInfo;
	}

	@Override
	public void setBrake(boolean brake) {
		NeutralModeValue neutralModeValue = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
		motor.setNeutralMode(neutralModeValue);
	}

	@Override
	public void resetAngle(Rotation2d angle) {
		motor.setPosition(angle.getRotations());
	}


	@Override
	public void stop() {
		motor.stopMotor();
	}

	@Override
	public void setPower(double power) {
		motor.setVoltage(power);
	}


	@Override
	public void setTargetVelocity(IControlRequest controlRequest) {
		TalonFXControlRequest talonFXRequest = (TalonFXControlRequest) controlRequest;
		motor.setControl(talonFXRequest.getControlRequest());
	}

	@Override
	public void setTargetAngle(IControlRequest controlRequest) {
		TalonFXControlRequest talonFXRequest = (TalonFXControlRequest) controlRequest;
		motor.setControl(talonFXRequest.getControlRequest());
	}


	@Override
	public void setTargetProfiledVelocity(IControlRequest controlRequest) {
		TalonFXControlRequest talonFXRequest = (TalonFXControlRequest) controlRequest;
		motor.setControl(talonFXRequest.getControlRequest());
	}

	@Override
	public void setTargetProfiledAngle(IControlRequest controlRequest) {
		TalonFXControlRequest talonFXRequest = (TalonFXControlRequest) controlRequest;
		motor.setControl(talonFXRequest.getControlRequest());
	}


	@Override
	public void updateInputs(MotorInputsAutoLogged motorInputs) {
		motorInputs.connected = BaseStatusSignal.refreshAll(signals.current(), signals.voltage()).isOK();
		motorInputs.current = signals.current().getValue();
		motorInputs.voltage = signals.voltage().getValue();
	}

	@Override
	public void updateInputs(PIDAbleInputsAutoLogged inputs) {
		BaseStatusSignal.refreshAll(signals.position(), signals.velocity(), signals.acceleration());
		inputs.angle = Rotation2d.fromRotations(motor.getLatencyCompensatedPosition());
		inputs.velocity = Rotation2d.fromRotations(motor.getLatencyCompensatedVelocity());
		inputs.acceleration = Rotation2d.fromRotations(signals.acceleration().getValue());
	}


}
