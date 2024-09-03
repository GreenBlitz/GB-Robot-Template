package frc.robot.hardware.motor.sparkmax;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.motor.IMotor;
import frc.robot.hardware.motor.MotorInputsAutoLogged;
import frc.robot.hardware.motor.PIDAble;
import frc.robot.hardware.motor.PIDAbleInputsAutoLogged;
import frc.robot.hardware.motor.ProfileAble;
import frc.robot.hardware.request.angle.IAngleRequest;
import frc.robot.hardware.request.angle.SparkMaxAngleRequest;
import frc.robot.hardware.request.value.IValueRequest;
import frc.robot.hardware.request.value.SparkMaxValueRequest;
import frc.utils.calibration.sysid.SysIdCalibrator;

public class SparkMaxMotor implements IMotor, PIDAble, ProfileAble {

	protected final CANSparkMax motor;
	protected final SparkMaxConstants constants;
	private Rotation2d lastVelocityPerSecond;

	public SparkMaxMotor(CANSparkMax motor, SparkMaxConstants constants) {
		this.motor = motor;
		this.constants = constants;
		this.lastVelocityPerSecond = new Rotation2d();
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
	public void setPower(double power) {
		motor.set(power);
	}


	@Override
	public void setVoltage(IValueRequest voltageRequest) {
		SparkMaxValueRequest sparkMaxVoltageRequest = (SparkMaxValueRequest) voltageRequest;
		motor.getPIDController()
			.setReference(sparkMaxVoltageRequest.getSetPoint(), CANSparkBase.ControlType.kVoltage, sparkMaxVoltageRequest.getSlot());
	}

	@Override
	public void setTargetVelocity(IAngleRequest velocityRequest) {
		SparkMaxAngleRequest sparkMaxVelocityRequest = (SparkMaxAngleRequest) velocityRequest;
		motor.getPIDController()
			.setReference(
				sparkMaxVelocityRequest.getSetPoint().getRotations(),
				CANSparkBase.ControlType.kVelocity,
				sparkMaxVelocityRequest.getSlot(),
				constants.feedforward()
					.apply(
						Rotation2d.fromRotations(motor.getEncoder().getPosition()),
						Rotation2d.fromRotations(motor.getEncoder().getVelocity())
					)
					.getRotations()
			);
	}

	@Override
	public void setTargetAngle(IAngleRequest angleRequest) {
		SparkMaxAngleRequest sparkMaxAngleRequest = (SparkMaxAngleRequest) angleRequest;
		motor.getPIDController()
			.setReference(
				sparkMaxAngleRequest.getSetPoint().getRotations(),
				CANSparkBase.ControlType.kPosition,
				sparkMaxAngleRequest.getSlot(),
				constants.feedforward()
					.apply(
						Rotation2d.fromRotations(motor.getEncoder().getPosition()),
						Rotation2d.fromRotations(motor.getEncoder().getVelocity())
					)
					.getRotations()
			);
	}


	@Override
	public void setTargetProfiledVelocity(IAngleRequest profiledVelocityRequest) {
		SparkMaxAngleRequest sparkMaxProfiledVelocityRequest = (SparkMaxAngleRequest) profiledVelocityRequest;
		motor.getPIDController()
			.setReference(
				sparkMaxProfiledVelocityRequest.getSetPoint().getRotations(),
				CANSparkBase.ControlType.kSmartVelocity,
				sparkMaxProfiledVelocityRequest.getSlot(),
				constants.feedforward()
					.apply(
						Rotation2d.fromRotations(motor.getEncoder().getPosition()),
						Rotation2d.fromRotations(motor.getEncoder().getVelocity())
					)
					.getRotations()
			);
	}

	@Override
	public void setTargetProfiledAngle(IAngleRequest profiledAngleRequest) {
		SparkMaxAngleRequest sparkMaxProfiledAngleRequest = (SparkMaxAngleRequest) profiledAngleRequest;
		motor.getPIDController()
			.setReference(
				sparkMaxProfiledAngleRequest.getSetPoint().getRotations(),
				CANSparkBase.ControlType.kSmartMotion,
				sparkMaxProfiledAngleRequest.getSlot(),
				constants.feedforward()
					.apply(
						Rotation2d.fromRotations(motor.getEncoder().getPosition()),
						Rotation2d.fromRotations(motor.getEncoder().getVelocity())
					)
					.getRotations()
			);
	}


	@Override
	public void updateInputs(MotorInputsAutoLogged motorInputs) {
		motorInputs.connected = true; // TODO: find a way to check it
		motorInputs.current = motor.getOutputCurrent();
		motorInputs.voltage = motor.getBusVoltage() * motor.getAppliedOutput();
	}

	@Override
	public void updateInputs(PIDAbleInputsAutoLogged pidAbleInputs) {
		pidAbleInputs.angle = Rotation2d.fromRotations(motor.getEncoder().getPosition());
		pidAbleInputs.velocity = Rotation2d.fromRotations(motor.getEncoder().getVelocity());
		pidAbleInputs.acceleration = Rotation2d.fromRotations(pidAbleInputs.velocity.getRotations() - lastVelocityPerSecond.getRotations());
		lastVelocityPerSecond = pidAbleInputs.velocity;
	}

}
