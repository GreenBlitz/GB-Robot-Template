package frc.robot.hardware.motor.talonfx;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hardware.motor.IMotor;
import frc.robot.hardware.motor.PIDAble;
import frc.robot.hardware.motor.ProfileAble;
import frc.robot.hardware.request.angle.IAngleRequest;
import frc.robot.hardware.request.value.IValueRequest;
import frc.robot.hardware.request.value.TalonFXValueRequest;
import frc.utils.calibration.sysid.SysIdCalibrator;
import frc.utils.devicewrappers.TalonFXWrapper;

public class TalonFXMotor implements IMotor, PIDAble, ProfileAble {

	protected final TalonFXWrapper motor;
	protected final SysIdCalibrator.SysIdConfigInfo sysidConfigInfo;

	public TalonFXMotor(TalonFXWrapper motor, SysIdRoutine.Config sysidConfig) {
		this.motor = motor;
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
	public void resetPosition(Rotation2d position) {
		motor.setPosition(position.getRotations());
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
	public void setTargetPosition(IAngleRequest positionRequest) {
		motor.setControl(((TalonFXValueRequest) positionRequest).getControlRequest());
	}


	@Override
	public void setTargetProfiledVelocity(IAngleRequest profiledVelocityRequest) {
		motor.setControl(((TalonFXValueRequest) profiledVelocityRequest).getControlRequest());
	}

	@Override
	public void setTargetProfiledPosition(IAngleRequest profiledPositionRequest) {
		motor.setControl(((TalonFXValueRequest) profiledPositionRequest).getControlRequest());
	}

}
