package frc.robot.hardware.cansparkmax;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.CloseLoopControl;
import frc.robot.hardware.ControlState;
import frc.robot.hardware.IMotor;
import frc.robot.hardware.MotorInputsAutoLogged;
import frc.utils.calibration.sysid.SysIdCalibrator;

import java.util.function.BiFunction;

public class SparkMaxMotor implements IMotor {

	protected final CANSparkMax motor;
	protected final BiFunction<Rotation2d, Rotation2d, Rotation2d> feedforward;
	protected final SparkMaxConstants constants;

	public SparkMaxMotor(CANSparkMax motor, SparkMaxConstants constants) {
		this.motor = motor;
		this.feedforward = constants.feedforward();
		this.constants = constants;
	}

	@Override
	public SysIdCalibrator.SysIdConfigInfo getSysidConfigInfo() {
		return constants.sysIdConfigInfo();
	}

	@Override
	public void setBrake(boolean brake) {
		CANSparkBase.IdleMode idleMode = brake ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast;
		motor.setIdleMode(idleMode);
	}

	@Override
	public void resetAngle(Rotation2d angle) {
		motor.getEncoder().setPosition(angle.getRotations());
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
		motor.getPIDController()
			.setReference(
				velocityControl.targetSetPoint().getRotations(),
				CANSparkBase.ControlType.kVelocity,
				velocityControl.pidSlot(),
				feedforward
					.apply(
						Rotation2d.fromRotations(motor.getEncoder().getPosition()),
						Rotation2d.fromRotations(motor.getEncoder().getVelocity())
					)
					.getRotations()
			);
	}

	@Override
	public void setTargetAngle(CloseLoopControl positionControl) {
		CANSparkBase.ControlType controlType = positionControl.controlState() == ControlState.MOTION_MAGIC
			? CANSparkBase.ControlType.kSmartMotion
			: CANSparkBase.ControlType.kPosition;
		motor.getPIDController()
			.setReference(
				positionControl.targetSetPoint().getRotations(),
				controlType,
				positionControl.pidSlot(),
				feedforward
					.apply(
						Rotation2d.fromRotations(motor.getEncoder().getPosition()),
						Rotation2d.fromRotations(motor.getEncoder().getVelocity())
					)
					.getRotations()
			);
	}

	@Override
	public void updateInputs(MotorInputsAutoLogged motorInputs) {
		motorInputs.connected = true;
		motorInputs.angle = Rotation2d.fromRotations(motor.getEncoder().getPosition());
		motorInputs.velocity = Rotation2d.fromRotations(motor.getEncoder().getVelocity());
		motorInputs.current = motor.getOutputCurrent();
		motorInputs.voltage = motor.getBusVoltage() * motor.getAppliedOutput();
	}

}
