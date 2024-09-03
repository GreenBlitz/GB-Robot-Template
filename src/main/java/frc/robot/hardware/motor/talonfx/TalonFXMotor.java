package frc.robot.hardware.motor.talonfx;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hardware.motor.IMotor;
import frc.robot.hardware.motor.MotorInputsAutoLogged;
import frc.robot.hardware.motor.PIDAble;
import frc.robot.hardware.motor.PIDAbleInputsAutoLogged;
import frc.robot.hardware.motor.ProfileAble;
import frc.robot.hardware.request.angle.IAngleRequest;
import frc.robot.hardware.request.value.IValueRequest;
import frc.robot.hardware.request.value.TalonFXValueRequest;
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
		motor.set(power);
	}


	@Override
	public void setVoltage(IValueRequest voltageRequest) {
		motor.setControl(((TalonFXValueRequest) voltageRequest).getControlRequest());
	}

	@Override
	public void setTargetVelocity(IAngleRequest velocityRequest) {
		motor.setControl(((TalonFXValueRequest) velocityRequest).getControlRequest());
	}

	@Override
	public void setTargetAngle(IAngleRequest angleRequest) {
		motor.setControl(((TalonFXValueRequest) angleRequest).getControlRequest());
	}


	@Override
	public void setTargetProfiledVelocity(IAngleRequest profiledVelocityRequest) {
		motor.setControl(((TalonFXValueRequest) profiledVelocityRequest).getControlRequest());
	}

	@Override
	public void setTargetProfiledAngle(IAngleRequest profiledAngleRequest) {
		motor.setControl(((TalonFXValueRequest) profiledAngleRequest).getControlRequest());
	}


	@Override
	public void updateInputs(MotorInputsAutoLogged motorInputs) {
		motorInputs.connected = BaseStatusSignal.refreshAll(signals.current(), signals.voltage()).isOK();
		motorInputs.current = signals.current().getValue();
		motorInputs.voltage = signals.voltage().getValue();
	}

	@Override
	public void updateInputs(PIDAbleInputsAutoLogged pidAbleInputs) {
		BaseStatusSignal.refreshAll(signals.position(), signals.velocity(), signals.acceleration());
		pidAbleInputs.angle = Rotation2d.fromRotations(motor.getLatencyCompensatedPosition());
		pidAbleInputs.velocity = Rotation2d.fromRotations(motor.getLatencyCompensatedVelocity());
		pidAbleInputs.acceleration = Rotation2d.fromRotations(signals.acceleration().getValue());
	}

}
