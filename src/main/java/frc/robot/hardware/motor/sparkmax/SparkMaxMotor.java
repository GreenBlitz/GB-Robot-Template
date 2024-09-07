package frc.robot.hardware.motor.sparkmax;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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

import java.util.function.BiFunction;

public class SparkMaxMotor implements IMotor, PIDAble, ProfileAble {

	protected final CANSparkMax motor;
	protected final BiFunction<Rotation2d, Rotation2d, Rotation2d> feedforward;
	protected final SysIdCalibrator.SysIdConfigInfo sysIdConfigInfo;
	private Rotation2d lastVelocityPerSecond;

	public SparkMaxMotor(CANSparkMax motor, BiFunction<Rotation2d, Rotation2d, Rotation2d> feedforward, SysIdRoutine.Config sysidConfig) {
		this.motor = motor;
		this.feedforward = feedforward;
		this.sysIdConfigInfo = new SysIdCalibrator.SysIdConfigInfo(sysidConfig, false);
		this.lastVelocityPerSecond = new Rotation2d();
	}

	@Override
	public SysIdCalibrator.SysIdConfigInfo getSysidConfigInfo() {
		return sysIdConfigInfo;
	}

	@Override
	public void setBrake(boolean brake) {
		CANSparkBase.IdleMode idleMode = brake ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast;
		motor.setIdleMode(idleMode);
	}

	@Override
	public void resetPosition(Rotation2d position) {
		motor.getEncoder().setPosition(position.getRotations());
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
				feedforward
					.apply(
						Rotation2d.fromRotations(motor.getEncoder().getPosition()),
						Rotation2d.fromRotations(motor.getEncoder().getVelocity())
					)
					.getRotations()
			);
	}

	@Override
	public void setTargetPosition(IAngleRequest positionRequest) {
		SparkMaxAngleRequest sparkMaxPositionRequest = (SparkMaxAngleRequest) positionRequest;
		motor.getPIDController()
			.setReference(
				sparkMaxPositionRequest.getSetPoint().getRotations(),
				CANSparkBase.ControlType.kPosition,
				sparkMaxPositionRequest.getSlot(),
				feedforward
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
				feedforward
					.apply(
						Rotation2d.fromRotations(motor.getEncoder().getPosition()),
						Rotation2d.fromRotations(motor.getEncoder().getVelocity())
					)
					.getRotations()
			);
	}

	@Override
	public void setTargetProfiledPosition(IAngleRequest profiledPositionRequest) {
		SparkMaxAngleRequest sparkMaxProfiledPositionRequest = (SparkMaxAngleRequest) profiledPositionRequest;
		motor.getPIDController()
			.setReference(
				sparkMaxProfiledPositionRequest.getSetPoint().getRotations(),
				CANSparkBase.ControlType.kSmartMotion,
				sparkMaxProfiledPositionRequest.getSlot(),
				feedforward
					.apply(
						Rotation2d.fromRotations(motor.getEncoder().getPosition()),
						Rotation2d.fromRotations(motor.getEncoder().getVelocity())
					)
					.getRotations() // TODO: find out what to give to ff (current or target setPoints)
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
		pidAbleInputs.position = Rotation2d.fromRotations(motor.getEncoder().getPosition());
		pidAbleInputs.velocity = Rotation2d.fromRotations(motor.getEncoder().getVelocity());
		pidAbleInputs.acceleration = Rotation2d.fromRotations(pidAbleInputs.velocity.getRotations() - lastVelocityPerSecond.getRotations());
		lastVelocityPerSecond = pidAbleInputs.velocity;
	}

}
