package frc.robot.hardware.mechanisms;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.utils.time.TimeUtils;

public class FlywheelSimulation extends MechanismSimulation {

	private final FlywheelSim flywheelSim;

	public FlywheelSimulation(FlywheelSim flywheelSim, double gearRatio) {
		super(gearRatio);
		this.flywheelSim = flywheelSim;
	}

	@Override
	public Rotation2d getSystemPosition() {
		return getSystemVelocityAnglesPerSecond().times(TimeUtils.getCurrentCycleTimeSeconds());
	}

	@Override
	public Rotation2d getSystemVelocityAnglesPerSecond() {
		return Rotation2d.fromRadians(flywheelSim.getAngularVelocityRadPerSec());
	}

	@Override
	public void setInputVoltage(double voltage) {
		flywheelSim.setInputVoltage(voltage);
	}

	@Override
	public void updateMotor() {
		flywheelSim.update(TimeUtils.getCurrentCycleTimeSeconds());
	}

}
