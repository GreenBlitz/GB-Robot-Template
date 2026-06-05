package frc.robot.hardware.mechanisms.wpilib;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.simulation.DCMotorSim;
import frc.utils.time.TimeUtil;

public class SimpleMotorSimulation implements WPILibMechanismSimulation {

	private final DCMotorSim motorSimulation;

	public SimpleMotorSimulation(DCMotorSim motorSimulation) {
		this.motorSimulation = motorSimulation;
	}

	@Override
	public Rotation2d getMechanismPosition() {
		return Rotation2d.fromRadians(motorSimulation.getAngularPosition());
	}

	@Override
	public Rotation2d getMechanismVelocityAnglesPerSecond() {
		return Rotation2d.fromRadians(motorSimulation.getAngularVelocity());
	}

	@Override
	public void setInputVoltage(double voltage) {
		motorSimulation.setInputVoltage(voltage);
	}

	@Override
	public void updateMotor() {
		motorSimulation.update(TimeUtil.getLatestCycleTimeSeconds());
	}

	@Override
	public double getGearRatio() {
		return motorSimulation.getGearing();
	}

}
