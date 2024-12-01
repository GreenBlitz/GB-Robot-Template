package frc.robot.hardware.mechanisms;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.utils.time.TimeUtils;

public class SimpleMotorSimulation extends MechanismSimulation {

	private final DCMotorSim motorSimulation;

	public SimpleMotorSimulation(DCMotorSim motorSimulation, double gearRatio) {
		super(gearRatio);
		this.motorSimulation = motorSimulation;
	}

	@Override
	public Rotation2d getSystemPosition() {
		return Rotation2d.fromRadians(motorSimulation.getAngularPositionRad());
	}

	@Override
	public Rotation2d getSystemVelocityAnglesPerSecond() {
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
