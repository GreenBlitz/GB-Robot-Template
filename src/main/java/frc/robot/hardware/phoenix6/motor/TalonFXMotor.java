package frc.robot.hardware.phoenix6.motor;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.phoenix6.Phoenix6Device;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.motor.simulation.TalonFXSimulation;
import frc.robot.hardware.mechanisms.MechanismSimulation;
import frc.robot.hardware.phoenix6.request.Phoenix6Request;
import frc.utils.alerts.Alert;
import frc.utils.calibration.sysid.SysIdCalibrator;

import java.util.Optional;

public class TalonFXMotor extends Phoenix6Device implements ControllableMotor {

	private static final int APPLY_CONFIG_RETRIES = 5;

	private final TalonFXWrapper motor;
	private final Optional<TalonFXSimulation> talonFXSimulationOptional;
	private final SysIdCalibrator.SysIdConfigInfo sysidConfigInfo;

	public TalonFXMotor(
		String logPath,
		Phoenix6DeviceID deviceID,
		TalonFXConfiguration configuration,
		SysIdRoutine.Config sysidConfig,
		MechanismSimulation simulation
	) {
		super(logPath);
		this.motor = new TalonFXWrapper(deviceID);
		applyConfiguration(configuration);
		this.talonFXSimulationOptional = createSimulation(simulation, configuration);
		this.sysidConfigInfo = new SysIdCalibrator.SysIdConfigInfo(sysidConfig, true);
		motor.optimizeBusUtilization();
	}

	public TalonFXMotor(String logPath, Phoenix6DeviceID deviceID, TalonFXConfiguration configuration, SysIdRoutine.Config sysidConfig) {
		this(logPath, deviceID, configuration, sysidConfig, null);
	}

	private void applyConfiguration(TalonFXConfiguration configuration) {
		if (!motor.applyConfiguration(configuration, APPLY_CONFIG_RETRIES).isOK()) {
			new Alert(Alert.AlertType.ERROR, getLogPath() + "ConfigurationFailed").report();
		}
	}

	private Optional<TalonFXSimulation> createSimulation(MechanismSimulation simulation, TalonFXConfiguration configuration) {
		return Robot.ROBOT_TYPE.isSimulation() && simulation != null
			? Optional.of(new TalonFXSimulation(motor, configuration, simulation))
			: Optional.empty();
	}

	@Override
	public TalonFXWrapper getDevice() {
		return motor;
	}

	@Override
	public void updateSimulation() {
		talonFXSimulationOptional.ifPresent(TalonFXSimulation::updateMotor);
	}

	@Override
	public SysIdCalibrator.SysIdConfigInfo getSysidConfigInfo() {
		return sysidConfigInfo;
	}

	@Override
	public void setBrake(boolean brake) {
		NeutralModeValue neutralModeValue = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
		motor.setNeutralMode(neutralModeValue);
	}

	@Override
	public void resetPosition(Rotation2d position) {
		motor.setPosition(position.getRotations());
	}

	@Override
	public void applyRequest(IRequest<?> request) {
		if (request instanceof Phoenix6Request<?> phoenix6Request) {
			motor.setControl(phoenix6Request.getControlRequest());
		} else {
			new Alert(Alert.AlertType.WARNING, getLogPath() + "Got invalid type of request " + request.getClass().getSimpleName()).report();
		}
	}

}
