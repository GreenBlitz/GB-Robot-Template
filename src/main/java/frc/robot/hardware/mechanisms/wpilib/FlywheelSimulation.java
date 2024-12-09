package frc.robot.hardware.mechanisms.wpilib;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.utils.time.TimeUtils;

public class FlywheelSimulation implements WPILibMechanismSimulation {

	private final FlywheelSim flywheelSimulation;
	private Rotation2d position;
	private Rotation2d currentVelocity;

	public FlywheelSimulation(FlywheelSim flywheelSimulation) {
		this.flywheelSimulation = flywheelSimulation;
		this.position = Rotation2d.fromDegrees(0);
		this.currentVelocity = getSystemVelocityAnglesPerSecond();
	}

	@Override
	public Rotation2d getSystemPosition() {
		return position;
	}

	@Override
	public Rotation2d getSystemVelocityAnglesPerSecond() {
		return Rotation2d.fromRadians(flywheelSimulation.getAngularVelocityRadPerSec());
	}

	@Override
	public void setInputVoltage(double voltage) {
		flywheelSimulation.setInputVoltage(voltage);
	}

	@Override
	public void updateMotor() {
		flywheelSimulation.update(TimeUtils.getCurrentCycleTimeSeconds());
		Rotation2d lastVelocity = currentVelocity;
		currentVelocity = getSystemVelocityAnglesPerSecond();
		Rotation2d averageVelocity = Rotation2d.fromRotations((currentVelocity.getRotations() + lastVelocity.getRotations()) / 2);
		updatePosition(averageVelocity);
	}

	@Override
	public double getGearRatio() {
		return flywheelSimulation.getGearing();
	}

	private void updatePosition(Rotation2d velocity) {
		Rotation2d deltaDistance = velocity.times(TimeUtils.getCurrentCycleTimeSeconds());
		position = Rotation2d.fromRotations(position.getRotations() + deltaDistance.getRotations());
	}

}
