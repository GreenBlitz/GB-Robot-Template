package frc.robot.hardware.mechanisms;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.utils.Conversions;
import frc.utils.time.TimeUtils;

public class ElevatorSimulation extends MechanismSimulation {

	private final ElevatorSim elevatorSimulation;
	private final double wheelDiameter;

	public ElevatorSimulation(ElevatorSim elevatorSimulation, double gearRatio, double wheelDiameter) {
		super(gearRatio);
		this.elevatorSimulation = elevatorSimulation;
		this.wheelDiameter = wheelDiameter;
	}

	@Override
	public Rotation2d getSystemPosition() {
		return Conversions.distanceToAngle(elevatorSimulation.getPositionMeters(), wheelDiameter);
	}

	@Override
	public Rotation2d getSystemVelocityAnglesPerSecond() {
		return Conversions.distanceToAngle(elevatorSimulation.getVelocityMetersPerSecond(), wheelDiameter);
	}

	@Override
	public void setInputVoltage(double voltage) {
		elevatorSimulation.setInputVoltage(voltage);
	}

	@Override
	public void updateMotor() {
		elevatorSimulation.update(TimeUtils.getCurrentCycleTimeSeconds());
	}

}
