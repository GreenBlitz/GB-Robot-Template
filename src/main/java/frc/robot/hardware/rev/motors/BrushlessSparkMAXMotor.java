package frc.robot.hardware.rev.motors;

import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.mechanisms.MechanismSimulation;
import frc.robot.hardware.rev.request.SparkMaxRequest;
import frc.utils.alerts.Alert;
import frc.utils.calibration.sysid.SysIdCalibrator;

public class BrushlessSparkMAXMotor extends SparkMaxMotor implements ControllableMotor {

	private final SysIdCalibrator.SysIdConfigInfo sysIdConfigInfo;

	public BrushlessSparkMAXMotor(
		String logPath,
		SparkMaxWrapper motor,
		MechanismSimulation mechanismSimulation,
		SysIdRoutine.Config sysidConfig
	) {
		super(logPath, motor, mechanismSimulation);
		if (motor.getMotorType() != SparkLowLevel.MotorType.kBrushless) {
			throw new IllegalArgumentException("inserted BrushedSparkMAXMotor to BrushlessSparkMAXMotor!");
		}
		this.sysIdConfigInfo = new SysIdCalibrator.SysIdConfigInfo(sysidConfig, false);
	}

	public BrushlessSparkMAXMotor(String logPath, SparkMaxWrapper motor, SysIdRoutine.Config sysidConfig) {
		this(logPath, motor, null, sysidConfig);
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
			motor.getClosedLoopController()
				.setReference(
					sparkMaxRequest.getSparkMaxCompatibleSetPoint(),
					sparkMaxRequest.getControlType(),
					sparkMaxRequest.getPidSlot(),
					sparkMaxRequest.getFeedforwardGain()
				);
		} else {
			new Alert(Alert.AlertType.WARNING, getLogPath() + "Got invalid type of request " + request.getClass().getSimpleName()).report();
		}
	}

}
