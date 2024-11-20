package frc.robot.hardware.mechanisms;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.utils.time.TimeUtils;

public class ElevatorSimulation extends MechanismSimulation {

	private final ElevatorSim elevatorSim;
	private final double metersToRotationsConversionFactor;

	public ElevatorSimulation(ElevatorSim elevatorSim, double gearRatio, double metersToRotationsConversionFactor) {
		super(gearRatio);
		this.elevatorSim = elevatorSim;
		this.metersToRotationsConversionFactor = metersToRotationsConversionFactor;
	}

	@Override
	public Rotation2d getSystemPosition() {
		return metersToRotations(elevatorSim.getPositionMeters());
	}

	@Override
	public Rotation2d getSystemVelocityAnglesPerSecond() {
		return metersToRotations(elevatorSim.getVelocityMetersPerSecond());
	}

	@Override
	public void setInputVoltage(double voltage) {
		elevatorSim.setInputVoltage(voltage);
	}

	public boolean hasHitLowerLimit() {
		return elevatorSim.hasHitLowerLimit();
	}

	public boolean hasHitUpperLimit() {
		return elevatorSim.hasHitUpperLimit();
	}

	public boolean wouldHitLowerLimit(Rotation2d position) {
		return elevatorSim.wouldHitLowerLimit(rotationsToMeters(position));
	}

	public boolean wouldHitUpperLimit(Rotation2d position) {
		return elevatorSim.wouldHitUpperLimit(rotationsToMeters(position));
	}

	@Override
	public void updateMotor() {
		elevatorSim.update(TimeUtils.getCurrentCycleTimeSeconds());
	}

	public Rotation2d metersToRotations(double meters) {
		return Rotation2d.fromRotations(meters * metersToRotationsConversionFactor);
	}

	public double rotationsToMeters(Rotation2d rotations) {
		return rotations.getRotations() / metersToRotationsConversionFactor;
	}

}
