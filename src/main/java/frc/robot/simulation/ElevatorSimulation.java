package frc.robot.simulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.utils.Conversions;
import frc.utils.time.TimeUtils;

public class ElevatorSimulation extends MotorSimulation {

	private final ElevatorSim elevatorSimulation;

	private final double diameterMeters;

	public ElevatorSimulation(ElevatorSim elevatorSimulation, double drumRadiusMeters) {
		this.elevatorSimulation = elevatorSimulation;
		this.diameterMeters = 2 * drumRadiusMeters;
	}

	/**
	 * Returns in Rotation2D the position of the drum.
	 */
	@Override
	public Rotation2d getPosition() {
		return Conversions.distanceToAngle(getPositionMeters(), diameterMeters);
	}

	public double getPositionMeters() {
		return elevatorSimulation.getPositionMeters();
	}

	/**
	 * Returns the velocity in Rotation2D of the drum.
	 */
	@Override
	public Rotation2d getVelocity() {
		return Conversions.distanceToAngle(getVelocityMetersPerSecond(), diameterMeters);
	}

	public double getVelocityMetersPerSecond() {
		return elevatorSimulation.getVelocityMetersPerSecond();
	}

	@Override
	protected void setInputVoltage(double voltage) {
		elevatorSimulation.setInputVoltage(voltage);
	}

	@Override
	protected void updateMotor() {
		elevatorSimulation.update(TimeUtils.getCurrentCycleTimeSeconds());
	}

}
