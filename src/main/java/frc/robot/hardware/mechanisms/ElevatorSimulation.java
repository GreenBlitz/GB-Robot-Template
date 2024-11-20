package frc.robot.hardware.mechanisms;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.utils.Conversions;
import frc.utils.time.TimeUtils;

public class ElevatorSimulation extends MechanismSimulation {

	private final ElevatorSim elevatorSim;
	private final double wheelDiameter;

	public ElevatorSimulation(ElevatorSim elevatorSim, double gearRatio, double wheelDiameter) {
		super(gearRatio);
		this.elevatorSim = elevatorSim;
		this.wheelDiameter = wheelDiameter;
	}

	@Override
	public Rotation2d getSystemPosition() {
		return Conversions.distanceToAngle(elevatorSim.getPositionMeters(), wheelDiameter);
	}

	@Override
	public Rotation2d getSystemVelocityAnglesPerSecond() {
		return Conversions.distanceToAngle(elevatorSim.getVelocityMetersPerSecond(), wheelDiameter);
	}

	@Override
	public void setInputVoltage(double voltage) {
		elevatorSim.setInputVoltage(voltage);
	}

	@Override
	public void updateMotor() {
		elevatorSim.update(TimeUtils.getCurrentCycleTimeSeconds());
	}

}
