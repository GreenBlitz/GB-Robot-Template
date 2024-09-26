package frc.robot.hardware.motor.sparkmax;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.request.IRequest;
import frc.robot.hardware.request.cansparkmax.SparkMaxAngleRequest;
import frc.robot.hardware.request.cansparkmax.SparkMaxDoubleRequest;
import frc.utils.calibration.sysid.SysIdCalibrator;

public class BrushlessSparkMAXMotor extends SparkMaxMotor implements ControllableMotor {

	private final SysIdCalibrator.SysIdConfigInfo sysIdConfigInfo;

	public BrushlessSparkMAXMotor(String logPath, CANSparkMax motor, SysIdRoutine.Config sysidConfig) {
		super(logPath, motor);
		if (motor.getMotorType() != CANSparkLowLevel.MotorType.kBrushless) {
			throw new IllegalArgumentException("inserted BrushedSparkMAXMotor to BrushlessSparkMAXMotor!");
		}
		this.sysIdConfigInfo = new SysIdCalibrator.SysIdConfigInfo(sysidConfig, false);
	}

	@Override
	public SysIdCalibrator.SysIdConfigInfo getSysidConfigInfo() {
		return sysIdConfigInfo;
	}

	@Override
	public void resetPosition(Rotation2d position) {
		motor.getEncoder().setPosition(position.getRotations());
	}

	//@formatter:off
	@Override
	public void applyDoubleRequest(IRequest<Double> request) {
		SparkMaxDoubleRequest doubleRequest = (SparkMaxDoubleRequest) request;
		motor.getPIDController().setReference(
				doubleRequest.getSetPoint(),
				doubleRequest.getControlType(),
				doubleRequest.getPidSlot()
		);
	}

	@Override
	public void applyAngleRequest(IRequest<Rotation2d> request) {
		SparkMaxAngleRequest angleRequest = (SparkMaxAngleRequest) request;
		motor.getPIDController().setReference(
				angleRequest.getSetPoint().getRotations(),
				angleRequest.getControlType(),
				angleRequest.getPidSlot(),
				angleRequest.getFeedforwardCalculator().apply(motor)
		);
	}
	//@formatter:on

}
