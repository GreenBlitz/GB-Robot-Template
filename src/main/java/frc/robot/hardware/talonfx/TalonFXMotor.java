package frc.robot.hardware.talonfx;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.ControlState;
import frc.robot.hardware.IMotor;
import frc.robot.hardware.MotorInputs;
import frc.utils.calibration.sysid.SysIdCalibrator;

public class TalonFXMotor implements IMotor {

	protected TalonFX mMotor;
	protected TalonFXSignals signals;
	PositionVoltage positionVoltage;
	MotionMagicExpoVoltage positionVoltageMagic;


	@Override
	public SysIdCalibrator.SysIdConfigInfo getSysIdConfigInfo() {
		return constants.getSysiD;
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
	public void setTargetVelocity(Rotation2d targetVelocity, ControlState controlState) {}

	@Override
	public void setTargetAngle(Rotation2d targetAngle, ControlState controlState) {
		switch (controlState) {
			case PID -> mMotor.setControl(positionVoltage.withPosition(targetAngle.getRotations()));
			case MOTION_MAGIC -> mMotor.setControl(positionVoltageMagic.withPosition(targetAngle.getRotations()));
		}
	}

	@Override
	public void setTargetAngle(Rotation2d targetAngle, ControlState controlState, int pidSlot) {
		switch (controlState) {
			case PID -> mMotor.setControl(positionVoltage.withPosition(targetAngle.getRotations()).withSlot(pidSlot));
			case MOTION_MAGIC -> mMotor.setControl(positionVoltageMagic.withPosition(targetAngle.getRotations()).withSlot(pidSlot));
		}
	}

	@Override
	public void updateInputs(MotorInputs motorInputs) {
		motorInputs.connected = BaseStatusSignal.refreshAll(signals.position(), signals.velocity(), signals.current(), signals.voltage()).isOK();
        motorInputs.angle = Rotation2d.fromRotations(signals.position().getValue());
        motorInputs.velocity = Rotation2d.fromRotations(signals.velocity().getValue());
        motorInputs.current = signals.current().getValue();
        motorInputs.voltage = signals.voltage().getValue();
	}

}
