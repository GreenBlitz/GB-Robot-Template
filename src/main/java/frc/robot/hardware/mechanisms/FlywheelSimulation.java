package frc.robot.hardware.mechanisms;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.utils.time.TimeUtils;

public class FlywheelSimulation extends MechanismSimulation {

	private final FlywheelSim flywheelSim;

	private Rotation2d position;

	public FlywheelSimulation(FlywheelSim flywheelSim, double gearRatio) {
		super(gearRatio);
		this.flywheelSim = flywheelSim;
		this.position = Rotation2d.fromRotations(0);
	}

	@Override
	public Rotation2d getSystemPosition() {
		return getAccelerationAnglesPerSecondSquared().times(TimeUtils.getCurrentCycleTimeSeconds());
	}

	@Override
	public Rotation2d getSystemVelocityAnglesPerSecond() {
		return Rotation2d.fromRadians(flywheelSim.getAngularVelocityRadPerSec());
	}

	public Rotation2d getVelocityRPM() {
		return Rotation2d.fromRotations(flywheelSim.getAngularVelocityRPM());
	}

	@Override
	public void setInputVoltage(double voltage) {
		flywheelSim.setInputVoltage(voltage);
	}

	public Rotation2d getAccelerationAnglesPerSecondSquared() {
		return Rotation2d.fromRadians(flywheelSim.getAngularAccelerationRadPerSecSq());
	}

	@Override
	public void updateMotor() {
		flywheelSim.update(TimeUtils.getCurrentCycleTimeSeconds());
	}

}
