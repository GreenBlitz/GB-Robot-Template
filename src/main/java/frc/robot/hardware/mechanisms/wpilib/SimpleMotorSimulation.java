package frc.robot.hardware.mechanisms.wpilib;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.utils.time.TimeUtil;

public class SimpleMotorSimulation implements WPILibMechanismSimulation {

	private final DCMotorSim motorSimulation;

	public SimpleMotorSimulation(DCMotorSim motorSimulation) {
		this.motorSimulation = motorSimulation;
	}

	@Override
	public Rotation2d getMechanismPosition() {
		return Rotation2d.fromRadians(motorSimulation.getAngularPositionRad());
	}

	@Override
	public Rotation2d getMechanismVelocityAnglesPerSecond() {
		return Rotation2d.fromRadians(motorSimulation.getAngularVelocityRadPerSec());
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
