package frc.robot.hardware.talonfx;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hardware.CloseLoopControl;
import frc.robot.hardware.IMotor;
import frc.robot.hardware.MotorInputsAutoLogged;
import frc.utils.calibration.sysid.SysIdCalibrator;
import frc.utils.devicewrappers.TalonFXWrapper;

public class TalonFXMotor implements IMotor {

	protected final TalonFXWrapper motor;
	protected final TalonFXSignals signals;
	protected final SysIdCalibrator.SysIdConfigInfo sysidConfigInfo;

	public TalonFXMotor(TalonFXWrapper motor, TalonFXSignals signals, SysIdRoutine.Config config) {
		this.motor = motor;
		this.signals = signals;
		this.sysidConfigInfo = new SysIdCalibrator.SysIdConfigInfo(config, true);
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
	public void setVoltage(double voltage) {
		motor.setVoltage(voltage);
	}

	@Override
	public void setTargetVelocity(CloseLoopControl velocityControl) {
		motor.setControl(velocityControl.controlRequest());
	}

	@Override
	public void setTargetAngle(CloseLoopControl positionControl) {
		motor.setControl(positionControl.controlRequest());
	}

	@Override
	public void updateInputs(MotorInputsAutoLogged motorInputs) {
		motorInputs.connected = BaseStatusSignal
			.refreshAll(signals.position(), signals.velocity(), signals.acceleration(), signals.current(), signals.voltage())
			.isOK();
		motorInputs.angle = Rotation2d.fromRotations(motor.getLatencyCompensatedPosition());
		motorInputs.velocity = Rotation2d.fromRotations(motor.getLatencyCompensatedVelocity());
		motorInputs.acceleration = Rotation2d.fromRotations(signals.acceleration().getValue());
		motorInputs.current = signals.current().getValue();
		motorInputs.voltage = signals.voltage().getValue();
	}

}
