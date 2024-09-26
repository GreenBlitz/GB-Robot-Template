package frc.robot.simulation;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.phoenix6.Phoenix6Device;
import frc.robot.hardware.request.IRequest;
import frc.robot.hardware.request.phoenix6.Phoenix6AngleRequest;
import frc.robot.hardware.request.phoenix6.Phoenix6DoubleRequest;
import frc.utils.battery.BatteryUtils;
import frc.robot.hardware.motor.phoenix6.TalonFXWrapper;
import frc.utils.calibration.sysid.SysIdCalibrator;


/**
 * A wrapper class for the WPILib default simulation classes, that'll act similarly to how the TalonFX motor controller works.
 */
abstract class MotorSimulation extends Phoenix6Device implements ControllableMotor {

	private final TalonFXWrapper motor;
	private DCMotorSim motorSim =

	private final TalonFXSimState motorSimulationState;

	protected MotorSimulation(String logPath) {
		super(logPath);
		SimulationManager.addSimulation(this);
		this.motor = SimulationManager.createNewMotorForSimulation();
		this.motorSimulationState = motor.getSimState();
		this.motorSimulationState.setSupplyVoltage(BatteryUtils.DEFAULT_VOLTAGE);
	}

	protected void updateSimulation() {
		setInputVoltage(motorSimulationState.getMotorVoltage());
		updateMotor();
		motorSimulationState.setRawRotorPosition(getPosition().getRotations());
		motorSimulationState.setRotorVelocity(getVelocity().getRotations());
	}

	public void applyConfiguration(TalonFXConfiguration config) {
		motor.applyConfiguration(config);
	}

	@Override
	public void setBrake(boolean brake) {
		NeutralModeValue modeValue = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
		motor.setNeutralMode(modeValue);
	}

	@Override
	public void resetPosition(Rotation2d position) {
		motor.setPosition(position.getRotations());
	}

	@Override
	public SysIdCalibrator.SysIdConfigInfo getSysidConfigInfo() {
		return new SysIdCalibrator.SysIdConfigInfo(new SysIdRoutine.Config(),false);
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

	public void setPower(double power) {
		motor.set(power);
	}

	public void stop() {
		motor.stopMotor();
	}

	public void setControl(ControlRequest request) {
		motor.setControl(request);
	}

	public double getVoltage() {
		return motor.getMotorVoltage().getValue();
	}

	public double getCurrent() {
		return motor.getStatorCurrent().getValue();
	}

	protected abstract void setInputVoltage(double voltage);

	protected abstract void updateMotor();

	public abstract Rotation2d getPosition();

	public abstract Rotation2d getVelocity();

}
