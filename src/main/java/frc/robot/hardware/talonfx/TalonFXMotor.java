package frc.robot.hardware.talonfx;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hardware.CloseLoopControl;
import frc.robot.hardware.IMotor;
import frc.robot.hardware.MotorInputs;
import frc.utils.calibration.sysid.SysIdCalibrator;
import frc.utils.devicewrappers.TalonFXWrapper;

public class TalonFXMotor implements IMotor {

	protected final TalonFXWrapper mMotor;
	protected final TalonFXSignals mSignals;
	protected final SysIdCalibrator.SysIdConfigInfo mSysidConfigInfo;

	public TalonFXMotor(TalonFXWrapper motor, TalonFXSignals signals, SysIdRoutine.Config config){
		this.mMotor = motor;
		this.mSignals = signals;
		this.mSysidConfigInfo = new SysIdCalibrator.SysIdConfigInfo(config, true);
	}

	@Override
	public SysIdCalibrator.SysIdConfigInfo getSysIdConfigInfo() {
		return mSysidConfigInfo;
	}

	@Override
	public void setBrake(boolean brake) {
		NeutralModeValue neutralModeValue = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
		mMotor.setNeutralMode(neutralModeValue);
	}

	@Override
	public void resetAngle(Rotation2d angle) {
		mMotor.setPosition(angle.getRotations());
	}

	@Override
	public void stop() {
		mMotor.stopMotor();
	}

	@Override
	public void setVoltage(double voltage) {
		mMotor.setVoltage(voltage);
	}

	@Override
	public void setTargetVelocity(CloseLoopControl velocityControl) {
		mMotor.setControl(velocityControl.controlRequest());
	}

	@Override
	public void setTargetAngle(CloseLoopControl positionControl) {
		mMotor.setControl(positionControl.controlRequest());
	}

	@Override
	public void updateInputs(MotorInputs motorInputs) {
		motorInputs.connected = BaseStatusSignal.refreshAll(mSignals.position(), mSignals.velocity(), mSignals.current(), mSignals.voltage()).isOK();
		motorInputs.angle = Rotation2d.fromRotations(mSignals.position().getValue());
		motorInputs.velocity = Rotation2d.fromRotations(mSignals.velocity().getValue());
		motorInputs.current = mSignals.current().getValue();
		motorInputs.voltage = mSignals.voltage().getValue();
	}

}
