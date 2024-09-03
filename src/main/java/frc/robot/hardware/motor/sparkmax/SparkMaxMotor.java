package frc.robot.hardware.motor.sparkmax;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.motor.IMotor;
import frc.robot.hardware.motor.MotorInputsAutoLogged;
import frc.robot.hardware.motor.PIDAble;
import frc.robot.hardware.motor.PIDAbleInputsAutoLogged;
import frc.robot.hardware.motor.ProfileAble;
import frc.robot.hardware.request.IControlRequest;
import frc.robot.hardware.request.SparkMaxControlRequest;
import frc.utils.calibration.sysid.SysIdCalibrator;

public class SparkMaxMotor implements IMotor, PIDAble, ProfileAble {

	protected final CANSparkMax motor;
	protected final SparkMaxConstants constants;
	private Rotation2d lastVelocity;

	public SparkMaxMotor(CANSparkMax motor, SparkMaxConstants constants) {
		this.motor = motor;
		this.constants = constants;
		this.lastVelocity = new Rotation2d();
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
	public void setVoltage(double voltage) {
		motor.setVoltage(voltage);
	}


	@Override
	public void setTargetVelocity(IControlRequest request) {
		SparkMaxControlRequest controlRequest = (SparkMaxControlRequest) request;
		motor.getPIDController()
			.setReference(
				controlRequest.getSetPoint().getRotations(),
				CANSparkBase.ControlType.kVelocity,
				controlRequest.getSlot(),
				constants.feedforward()
					.apply(
						Rotation2d.fromRotations(motor.getEncoder().getPosition()),
						Rotation2d.fromRotations(motor.getEncoder().getVelocity())
					)
					.getRotations()
			);
	}

	@Override
	public void setTargetAngle(IControlRequest request) {
		SparkMaxControlRequest controlRequest = (SparkMaxControlRequest) request;
		motor.getPIDController()
			.setReference(
				controlRequest.getSetPoint().getRotations(),
				CANSparkBase.ControlType.kPosition,
				controlRequest.getSlot(),
				constants.feedforward()
					.apply(
						Rotation2d.fromRotations(motor.getEncoder().getPosition()),
						Rotation2d.fromRotations(motor.getEncoder().getVelocity())
					)
					.getRotations()
			);
	}


	@Override
	public void setTargetProfiledVelocity(IControlRequest request) {
		SparkMaxControlRequest controlRequest = (SparkMaxControlRequest) request;
		motor.getPIDController()
			.setReference(
				controlRequest.getSetPoint().getRotations(),
				CANSparkBase.ControlType.kSmartVelocity,
				controlRequest.getSlot(),
				constants.feedforward()
					.apply(
						Rotation2d.fromRotations(motor.getEncoder().getPosition()),
						Rotation2d.fromRotations(motor.getEncoder().getVelocity())
					)
					.getRotations()
			);
	}

	@Override
	public void setTargetProfiledAngle(IControlRequest request) {
		SparkMaxControlRequest controlRequest = (SparkMaxControlRequest) request;
		motor.getPIDController()
			.setReference(
				controlRequest.getSetPoint().getRotations(),
				CANSparkBase.ControlType.kSmartMotion,
				controlRequest.getSlot(),
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
		motorInputs.connected = true;
		motorInputs.current = motor.getOutputCurrent();
		motorInputs.voltage = motor.getBusVoltage() * motor.getAppliedOutput();
	}

	@Override
	public void updateInputs(PIDAbleInputsAutoLogged inputs) {
		inputs.angle = Rotation2d.fromRotations(motor.getEncoder().getPosition());
		inputs.velocity = Rotation2d.fromRotations(motor.getEncoder().getVelocity());
		inputs.acceleration = Rotation2d.fromRotations(inputs.velocity.getRotations() - lastVelocity.getRotations());
		lastVelocity = inputs.velocity;
	}

}
