package frc.robot.hard.rev.motors;

import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hard.interfaces.ControllableMotor;
import frc.robot.hard.interfaces.IRequest;
import frc.robot.hard.rev.request.SparkMaxRequest;
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
	public void applyRequest(IRequest<?> request) {
		if (request instanceof SparkMaxRequest<?> sparkMaxRequest) {
			motor.getPIDController()
				.setReference(
					sparkMaxRequest.getSetPointAsDouble(),
					sparkMaxRequest.getControlType(),
					sparkMaxRequest.getPidSlot(),
					sparkMaxRequest.getFeedforwardGain()
				);
		} else {
			new Alert(Alert.AlertType.WARNING, getLogPath() + "Got invalid type of request " + request.getClass().getSimpleName()).report();
		}
	}

}
