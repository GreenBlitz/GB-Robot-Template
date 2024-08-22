package frc.robot.hardware.cansparkmax;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.ControlState;
import frc.robot.hardware.IMotor;
import frc.robot.hardware.MotorInputs;
import frc.utils.calibration.sysid.SysIdCalibrator;

import java.util.function.BiFunction;

public class CanSparkMaxMotor implements IMotor {

	CANSparkMax mCanSparkMax;
	BiFunction<Rotation2d, Rotation2d, Rotation2d> ff;
	ConstantsNeo constants;

	@Override
	public SysIdCalibrator.SysIdConfigInfo getSysIdConfigInfo() {
		return constants.getSysiD;
	}

	@Override
	public void setBrake(boolean brake) {
		CANSparkBase.IdleMode idleMode = brake ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast;
		mCanSparkMax.setIdleMode(idleMode);
	}

	@Override
	public void resetAngle(Rotation2d angle) {
		mCanSparkMax.getEncoder().setPosition(angle.getRotations());
	}

	@Override
	public void stop() {
		mCanSparkMax.stopMotor();
	}

	@Override
	public void setVoltage(double voltage) {
		mCanSparkMax.setVoltage(voltage);
	}

	@Override
	public void setTargetVelocity(Rotation2d targetVelocity, ControlState controlState) {
		mCanSparkMax.getPIDController()
			.setReference(
				targetVelocity.getRotations(),
				CANSparkBase.ControlType.kVelocity,
				constants.getDefVelocitySlot(),
				ff.apply(
					Rotation2d.fromRotations(mCanSparkMax.getEncoder().getPosition()),
					Rotation2d.fromRotations(mCanSparkMax.getEncoder().getVelocity())
				).getRotations()
			);
	}

	@Override
	public void setTargetAngle(Rotation2d targetAngle, ControlState controlState) {
		setTargetAngle(targetAngle, controlState, constants.getDefPosSlot());
	}

	@Override
	public void setTargetAngle(Rotation2d targetAngle, ControlState controlState, int pidSlot) {
		CANSparkBase.ControlType controlType = controlState == ControlState.PID
			? CANSparkBase.ControlType.kPosition
			: CANSparkBase.ControlType.kSmartMotion;
		mCanSparkMax.getPIDController()
			.setReference(
				targetAngle.getRotations(),
				CANSparkBase.ControlType.kVelocity,
				pidSlot,
				ff.apply(
					Rotation2d.fromRotations(mCanSparkMax.getEncoder().getPosition()),
					Rotation2d.fromRotations(mCanSparkMax.getEncoder().getVelocity())
				).getRotations()
			);
	}

	@Override
	public void updateInputs(MotorInputs motorInputs) {
		motorInputs.connected = true;
		motorInputs.angle = Rotation2d.fromRotations(mCanSparkMax.getEncoder().getPosition());
		motorInputs.velocity = Rotation2d.fromRotations(mCanSparkMax.getEncoder().getVelocity());
		motorInputs.current = mCanSparkMax.getOutputCurrent();
		motorInputs.voltage = mCanSparkMax.getBusVoltage() * mCanSparkMax.getAppliedOutput();
	}

}
