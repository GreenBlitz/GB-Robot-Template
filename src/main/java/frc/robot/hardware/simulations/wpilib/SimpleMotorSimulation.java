package frc.robot.hardware.simulations.wpilib;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.utils.angle2d.Angle2d;
import frc.utils.time.TimeUtil;

public class SimpleMotorSimulation implements WPILibMechanismSimulation {

	private final DCMotorSim motorSimulation;

	public SimpleMotorSimulation(DCMotorSim motorSimulation) {
		this.motorSimulation = motorSimulation;
	}

	@Override
	public Angle2d getMechanismPosition() {
		return Angle2d.fromRadians(motorSimulation.getAngularPositionRad());
	}

	@Override
	public Angle2d getMechanismVelocityAPS() {
		return Angle2d.fromRadians(motorSimulation.getAngularVelocityRadPerSec());
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
