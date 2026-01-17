package frc.robot.hardware.phoenix6.motors;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.hardware.FollowerInputs;
import frc.robot.hardware.FollowerInputsAutoLogged;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IMotionMagicRequest;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.hardware.mechanisms.MechanismSimulation;
import frc.robot.hardware.phoenix6.Phoenix6Device;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.phoenix6.motors.simulation.TalonFXSimulation;
import frc.robot.hardware.phoenix6.request.Phoenix6Request;
import frc.utils.alerts.Alert;
import frc.utils.alerts.AlertManager;
import frc.utils.alerts.PeriodicAlert;
import frc.utils.calibration.sysid.SysIdCalibrator;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

public class TalonFXMotor extends Phoenix6Device implements ControllableMotor {

	private static final int APPLY_CONFIG_RETRIES = 5;

	private final TalonFXWrapper motor;
	private final TalonFXWrapper[] followers;
	private final FollowerInputsAutoLogged[] followerInputs;
	private final TalonFXFollowerConfig followerConfig;
	private final Optional<TalonFXSimulation> talonFXSimulationOptional;
	private final SysIdCalibrator.SysIdConfigInfo sysidConfigInfo;

	public TalonFXMotor(
		String logPath,
		Phoenix6DeviceID deviceID,
		TalonFXFollowerConfig followerConfig,
		SysIdRoutine.Config sysidConfig,
		MechanismSimulation simulation
	) {
		super(logPath);
		this.motor = new TalonFXWrapper(deviceID);
		this.followers = initializeFollowers(motor, followerConfig);
		this.followerInputs = initializeFollowerInputs(getLogPath(), followerConfig, followers.length);
		this.followerConfig = followerConfig;
		this.talonFXSimulationOptional = createSimulation(simulation);
		this.sysidConfigInfo = new SysIdCalibrator.SysIdConfigInfo(sysidConfig, true);

		for (TalonFXWrapper follower : followers) {
			applyConfiguration(follower, followerConfig.motorConfig);
		}
		motor.optimizeBusUtilization();
	}

	public TalonFXMotor(String logPath, Phoenix6DeviceID deviceID, TalonFXFollowerConfig followerConfig, SysIdRoutine.Config sysidConfig) {
		this(logPath, deviceID, followerConfig, sysidConfig, null);
	}

	public TalonFXMotor(String logPath, Phoenix6DeviceID deviceID, SysIdRoutine.Config sysidConfig, MechanismSimulation mechanismSimulation) {
		this(logPath, deviceID, null, sysidConfig, mechanismSimulation);
	}

	public TalonFXMotor(String logPath, Phoenix6DeviceID deviceID, SysIdRoutine.Config sysidConfig) {
		this(logPath, deviceID, null, sysidConfig, null);
	}

	private static TalonFXWrapper[] initializeFollowers(TalonFXWrapper main, TalonFXFollowerConfig followerConfig) {
		TalonFXWrapper[] followers = new TalonFXWrapper[followerConfig == null ? 0 : followerConfig.followerIDs.length];

		for (int i = 0; i < followers.length; i++) {
			followers[i] = new TalonFXWrapper(followerConfig.followerIDs[i].id());
			followers[i].setControl(new Follower(main.getDeviceID(), followerConfig.followerIDs[i].motorAlignmentValue()));
			BaseStatusSignal.setUpdateFrequencyForAll(
				RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
				followers[i].getPosition(),
				followers[i].getMotorVoltage()
			);
			followers[i].optimizeBusUtilization();
		}

		return followers;
	}

	private static FollowerInputsAutoLogged[] initializeFollowerInputs(String logPath, TalonFXFollowerConfig followerConfig, int followersNum) {
		FollowerInputsAutoLogged[] followerInputs = new FollowerInputsAutoLogged[followersNum];

		for (int i = 0; i < followerInputs.length; i++) {
			String followerLogPath = logPath + "/followers/" + followerConfig.followerIDs[i].name();
			FollowerInputsAutoLogged followerInputsAutoLogged = new FollowerInputsAutoLogged();
			followerInputs[i] = followerInputsAutoLogged;
			AlertManager.addAlert(
				new PeriodicAlert(
					Alert.AlertType.ERROR,
					followerLogPath + "disconnectedAt",
					() -> !followerInputsAutoLogged.followerData.connected()
				)
			);
		}

		return followerInputs;
	}

	public void applyConfiguration(TalonFXConfiguration configuration) {
		applyConfiguration(motor, configuration);
	}

	private void applyConfiguration(TalonFXWrapper motor, TalonFXConfiguration configuration) {
		if (talonFXSimulationOptional.isPresent()) {
			talonFXSimulationOptional.get().applyConfig(motor, configuration);
		} else if (!motor.applyConfiguration(configuration, APPLY_CONFIG_RETRIES).isOK()) {
			new Alert(Alert.AlertType.ERROR, getLogPath() + "ConfigurationFailed").report();
		}
	}

	private Optional<TalonFXSimulation> createSimulation(MechanismSimulation simulation) {
		return Robot.ROBOT_TYPE.isSimulation() && simulation != null
			? Optional.of(new TalonFXSimulation(motor, simulation, followers))
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
	public void updateInputs(InputSignal<?>... inputSignals) {
		super.updateInputs(inputSignals);
		for (int i = 0; i < followers.length; i++) {
			String followerLogPath = getLogPath() + "/followers/" + followerConfig.followerIDs[i].name();
			followerInputs[i].followerData = new FollowerInputs.FollowerData(
				followers[i].isConnected(),
				new Rotation2d(followers[i].getPosition().getValue()),
				followers[i].getMotorVoltage().getValueAsDouble()
			);
			Logger.processInputs(followerLogPath, followerInputs[i]);
		}
	}

	@Override
	public SysIdCalibrator.SysIdConfigInfo getSysidConfigInfo() {
		return sysidConfigInfo;
	}

	@Override
	public void setBrake(boolean brake) {
		NeutralModeValue neutralModeValue = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
		motor.setNeutralMode(neutralModeValue);
		for (TalonFXWrapper follower : followers) {
			follower.setNeutralMode(neutralModeValue);
		}
	}

	@Override
	public void resetPosition(Rotation2d position) {
		motor.setPosition(position.getRotations());
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
	public void applyRequest(IRequest<?> request) {
		if (request instanceof Phoenix6Request<?> phoenix6Request) {
			if (phoenix6Request instanceof IMotionMagicRequest) {
				motor.stopMotor();
			}
			motor.setControl(phoenix6Request.getControlRequest());
		} else {
			new Alert(Alert.AlertType.WARNING, getLogPath() + "Got invalid type of request " + request.getClass().getSimpleName()).report();
		}
	}

}
