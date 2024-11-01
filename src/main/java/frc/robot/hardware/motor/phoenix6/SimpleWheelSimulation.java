package frc.robot.hardware.motor.phoenix6;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.utils.time.TimeUtils;

public class SimpleWheelSimulation extends MechanismSimulation {

	private final DCMotorSim motorSimulation;

	public SimpleWheelSimulation(DCMotor gearbox, double gearRatio, double momentOfInertia) {
		super(gearRatio);
		motorSimulation = new DCMotorSim(gearbox, gearRatio, momentOfInertia);
	}

	@Override
	public Rotation2d getSystemPositionRotations() {
		return Rotation2d.fromRadians(motorSimulation.getAngularPositionRad());
	}

	@Override
	public Rotation2d getSystemVelocityRotationsPerSecond() {
		return Rotation2d.fromRadians(motorSimulation.getAngularVelocityRadPerSec());
	}

	@Override
	public void setInputVoltage(double voltage) {
		motorSimulation.setInputVoltage(voltage);
	}

	@Override
	public void updateMotor() {
		motorSimulation.update(TimeUtils.getCurrentCycleTimeSeconds());
	}

}
