package frc.robot.hardware.phoenix6.motors;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.RobotConstants;
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
import frc.utils.calibration.sysid.SysIdCalibrator;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

public class TalonFXMotor extends Phoenix6Device implements ControllableMotor {

	private static final int APPLY_CONFIG_RETRIES = 5;

	private final TalonFXWrapper motor;
	private final TalonFXWrapper[] followers;
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
		this.talonFXSimulationOptional = createSimulation(simulation);
		this.sysidConfigInfo = new SysIdCalibrator.SysIdConfigInfo(sysidConfig, true);
		this.followerConfig = followerConfig;

		motor.optimizeBusUtilization();

		if (followerConfig == null) {
			followers = new TalonFXWrapper[0];
		} else {
			followers = new TalonFXWrapper[followerConfig.followerIDs.length];
		}

		initializeFollowers();
	}

	private void initializeFollowers() {
		for (int i = 0; i < followers.length; i++) {
			followers[i] = new TalonFXWrapper(followerConfig.followerIDs[i].id());
			applyConfiguration(followers[i], followerConfig.motorConfig);
			followers[i].setControl(new Follower(motor.getDeviceID(), followerConfig.followerIDs[i].opposeMain()));
			BaseStatusSignal.setUpdateFrequencyForAll(
				RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
				followers[i].getPosition(),
				followers[i].getMotorVoltage()
			);
			followers[i].optimizeBusUtilization();
		}
	}

	public TalonFXMotor(String logPath, Phoenix6DeviceID deviceID, SysIdRoutine.Config sysidConfig, MechanismSimulation mechanismSimulation) {
		this(logPath, deviceID, null, sysidConfig, mechanismSimulation);
	}

	public TalonFXMotor(String logPath, Phoenix6DeviceID deviceID, SysIdRoutine.Config sysidConfig) {
		this(logPath, deviceID, null, sysidConfig, null);
	}

	private void applyConfiguration(TalonFXWrapper motor, TalonFXConfiguration configuration) {
		if (talonFXSimulationOptional.isPresent()) {
			talonFXSimulationOptional.get().applyConfig(motor, configuration);
		} else if (!motor.applyConfiguration(configuration, APPLY_CONFIG_RETRIES).isOK()) {
			new Alert(Alert.AlertType.ERROR, getLogPath() + "ConfigurationFailed").report();
		}
	}

	public void applyConfiguration(TalonFXConfiguration configuration) {
		applyConfiguration(motor, configuration);
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
			Logger.recordOutput(followerLogPath + "/position", followers[i].getPosition().getValue().in(Units.Radians));
			Logger.recordOutput(followerLogPath + "/voltage", followers[i].getMotorVoltage().getValueAsDouble());
			Logger.recordOutput(followerLogPath + "/connected", followers[i].isConnected());
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
