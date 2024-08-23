package frc.robot.hardware.cansparkmax;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.CloseLoopControl;
import frc.robot.hardware.ControlState;
import frc.robot.hardware.IMotor;
import frc.robot.hardware.MotorInputs;
import frc.robot.hardware.MotorInputsAutoLogged;
import frc.utils.calibration.sysid.SysIdCalibrator;

import java.util.function.BiFunction;

public class SparkMaxMotor implements IMotor {

	protected final CANSparkMax mMotor;
	protected final BiFunction<Rotation2d, Rotation2d, Rotation2d> mFeedforward;
	protected final SparkMaxConstants mConstants;

	public SparkMaxMotor(CANSparkMax canSparkMax, BiFunction<Rotation2d, Rotation2d, Rotation2d> feedforward, SparkMaxConstants mConstants) {
		this.mMotor = canSparkMax;
		this.mFeedforward = feedforward;
		this.mConstants = mConstants;
	}

	@Override
	public SysIdCalibrator.SysIdConfigInfo getSysIdConfigInfo() {
		return mConstants.sysIdConfigInfo();
	}

	@Override
	public void setBrake(boolean brake) {
		CANSparkBase.IdleMode idleMode = brake ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast;
		mMotor.setIdleMode(idleMode);
	}

	@Override
	public void resetAngle(Rotation2d angle) {
		mMotor.getEncoder().setPosition(angle.getRotations());
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
		mMotor.getPIDController()
			.setReference(
				velocityControl.targetSetPoint().getRotations(),
				CANSparkBase.ControlType.kVelocity,
				velocityControl.pidSlot(),
				mFeedforward
					.apply(
						Rotation2d.fromRotations(mMotor.getEncoder().getPosition()),
						Rotation2d.fromRotations(mMotor.getEncoder().getVelocity())
					)
					.getRotations()
			);
	}

	@Override
	public void setTargetAngle(CloseLoopControl positionControl) {
		CANSparkBase.ControlType controlType = positionControl.controlState() == ControlState.MOTION_MAGIC
			? CANSparkBase.ControlType.kSmartMotion
			: CANSparkBase.ControlType.kPosition;
		mMotor.getPIDController()
			.setReference(
				positionControl.targetSetPoint().getRotations(),
				controlType,
				positionControl.pidSlot(),
				mFeedforward
					.apply(
						Rotation2d.fromRotations(mMotor.getEncoder().getPosition()),
						Rotation2d.fromRotations(mMotor.getEncoder().getVelocity())
					)
					.getRotations()
			);
	}

	@Override
	public void updateInputs(MotorInputsAutoLogged motorInputs) {
		motorInputs.connected = true;
		motorInputs.angle = Rotation2d.fromRotations(mMotor.getEncoder().getPosition());
		motorInputs.velocity = Rotation2d.fromRotations(mMotor.getEncoder().getVelocity());
		motorInputs.current = mMotor.getOutputCurrent();
		motorInputs.voltage = mMotor.getBusVoltage() * mMotor.getAppliedOutput();
	}

}
