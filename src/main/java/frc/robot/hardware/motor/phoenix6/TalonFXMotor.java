package frc.robot.hardware.motor.phoenix6;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.phoenix6.Phoenix6Device;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;
import frc.robot.hardware.request.IRequest;
import frc.robot.hardware.request.phoenix6.Phoenix6AngleRequest;
import frc.robot.hardware.request.phoenix6.Phoenix6DoubleRequest;
import frc.utils.calibration.sysid.SysIdCalibrator;

public class TalonFXMotor extends Phoenix6Device implements ControllableMotor {

	private final TalonFXWrapper motor;
	private final TalonFXSimulation talonFXSimulation;
	private final SysIdCalibrator.SysIdConfigInfo sysidConfigInfo;



	public TalonFXMotor(
		String logPath,
		Phoenix6DeviceID deviceID,
		TalonFXConfiguration configuration,
		SysIdRoutine.Config sysidConfig,
		MechanismSimulation simulation
	) {
		super(logPath);
		this.motor = Robot.ROBOT_TYPE.isSimulation() ? new TalonFXWrapper(deviceID.ID()) : new TalonFXWrapper(deviceID);
		motor.applyConfiguration(configuration, 5);
		this.talonFXSimulation = Robot.ROBOT_TYPE.isSimulation() ? new TalonFXSimulation(motor, configuration, simulation) : null;
		this.sysidConfigInfo = new SysIdCalibrator.SysIdConfigInfo(sysidConfig, true);
		motor.optimizeBusUtilization();
	}

	public TalonFXSimulation getTalonFXSimulation() {
		return talonFXSimulation;
	}

	public TalonFXMotor(String logPath, Phoenix6DeviceID deviceID, TalonFXConfiguration configuration, MechanismSimulation simulation) {
		this(logPath, deviceID, configuration, new SysIdRoutine.Config(), simulation);
	}

	public TalonFXWrapper getMotor() {
		return motor;
	}

	@Override
	public void updateSimulation() {
		if (talonFXSimulation != null) {
			talonFXSimulation.updateMotor();
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
	public void applyDoubleRequest(IRequest<Double> request) {
		if (request instanceof Phoenix6DoubleRequest) {
			motor.setControl(((Phoenix6DoubleRequest) request).getControlRequest());
		}
	}

	@Override
	public void applyAngleRequest(IRequest<Rotation2d> request) {
		if (request instanceof Phoenix6AngleRequest) {
			motor.setControl(((Phoenix6AngleRequest) request).getControlRequest());
		}
	}

}
