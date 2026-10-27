package frc.robot.hardware.simulations.wpilib;

import org.wpilib.math.geometry.Angle2d;
import org.wpilib.simulation.FlywheelSim;
import frc.utils.time.TimeUtil;

public class FlywheelSimulation implements WPILibMechanismSimulation {

	private final FlywheelSim flywheelSimulation;
	private Angle2d currentPosition;
	private Angle2d currentVelocity;

	public FlywheelSimulation(FlywheelSim flywheelSimulation) {
		this.flywheelSimulation = flywheelSimulation;
		this.currentPosition = Angle2d.fromDegrees(0);
		this.currentVelocity = getMechanismVelocityRPS();
	}

	@Override
	public Angle2d getMechanismPosition() {
		return currentPosition;
	}

	@Override
	public Angle2d getMechanismVelocityRPS() {
		return Angle2d.fromRadians(flywheelSimulation.getAngularVelocity());
	}

	@Override
	public void setInputVoltage(double voltage) {
		flywheelSimulation.setInputVoltage(voltage);
	}

	@Override
	public void updateMotor() {
		flywheelSimulation.update(TimeUtil.getLatestCycleTimeSeconds());
		Angle2d lastVelocity = currentVelocity;
		currentVelocity = getMechanismVelocityRPS();
		Angle2d averageVelocity = Angle2d.fromRotations((currentVelocity.getRotations() + lastVelocity.getRotations()) / 2);
		updatePosition(averageVelocity);
	}

	@Override
	public double getGearRatio() {
		return flywheelSimulation.getGearing();
	}

	private void updatePosition(Angle2d velocity) {
		Angle2d deltaDistance = velocity.times(TimeUtil.getLatestCycleTimeSeconds());
		currentPosition = Angle2d.fromRotations(currentPosition.getRotations() + deltaDistance.getRotations());
	}

}
