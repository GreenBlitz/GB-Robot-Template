package frc.robot.hardware.motor.sparkmax;

import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.request.IRequest;
import frc.robot.hardware.request.cansparkmax.SparkMaxAngleRequest;
import frc.robot.hardware.request.cansparkmax.SparkMaxDoubleRequest;
import frc.utils.alerts.Alert;
import frc.utils.calibration.sysid.SysIdCalibrator;

public class BrushlessSparkMAXMotor extends SparkMaxMotor implements ControllableMotor {

	private final SysIdCalibrator.SysIdConfigInfo sysIdConfigInfo;

	public BrushlessSparkMAXMotor(String logPath, SparkMaxWrapper motor, SysIdRoutine.Config sysidConfig) {
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

	@Override
	public void applyRequest(IRequest request) {
		if (request instanceof SparkMaxDoubleRequest sparkDoubleRequest) {
			applyDoubleRequest(sparkDoubleRequest);
		} else if (request instanceof SparkMaxAngleRequest sparkMaxAngleRequest) {
			applyAngleRequest(sparkMaxAngleRequest);
		} else {
			new Alert(Alert.AlertType.WARNING, getLogPath() + "got invalid type of request").report();
		}
	}

	//@formatter:off
	public void applyDoubleRequest(SparkMaxDoubleRequest doubleRequest) {
		motor.getPIDController().setReference(
			doubleRequest.getSetPoint(),
			doubleRequest.getControlType(),
			doubleRequest.getPidSlot()
		);
	}

	public void applyAngleRequest(SparkMaxAngleRequest angleRequest) {
		motor.getPIDController().setReference(
			angleRequest.getSetPoint().getRotations(),
			angleRequest.getControlType(),
			angleRequest.getPidSlot(),
			angleRequest.getFeedforwardCalculation()
		);
	}
	//@formatter:on

}
