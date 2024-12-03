package frc.robot.hardware.mechanisms.wpilib;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.utils.Conversions;
import frc.utils.time.TimeUtils;

public class ElevatorSimulation implements WPILibMechanismSimulation {

	private final ElevatorSim elevatorSimulation;
	private final double drumDiameter;
	private final double gearRatio;

	public ElevatorSimulation(ElevatorSim elevatorSimulation, double drumDiameter, double gearRatio) {
		this.elevatorSimulation = elevatorSimulation;
		this.drumDiameter = drumDiameter;
		this.gearRatio = gearRatio;
	}

	@Override
	public Rotation2d getSystemPosition() {
		return Conversions.distanceToAngle(elevatorSimulation.getPositionMeters(), drumDiameter);
	}

	@Override
	public Rotation2d getSystemVelocityAnglesPerSecond() {
		return Conversions.distanceToAngle(elevatorSimulation.getVelocityMetersPerSecond(), drumDiameter);
	}

	@Override
	public void setInputVoltage(double voltage) {
		elevatorSimulation.setInputVoltage(voltage);
	}

	@Override
	public void updateMotor() {
		elevatorSimulation.update(TimeUtils.getCurrentCycleTimeSeconds());
	}

	@Override
	public double getGearRatio() {
		return gearRatio;
	}

}
