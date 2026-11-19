package frc.robot.hardware.simulations.wpilib;

import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.utils.Conversions;
import frc.utils.angle2d.Angle2d;
import frc.utils.time.TimeUtil;

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
	public Angle2d getMechanismPosition() {
		return Conversions.distanceToAngle(elevatorSimulation.getPositionMeters(), drumDiameter);
	}

	@Override
	public Angle2d getMechanismVelocityAPS() {
		return Conversions.distanceToAngle(elevatorSimulation.getVelocityMetersPerSecond(), drumDiameter);
	}

	@Override
	public void setInputVoltage(double voltage) {
		elevatorSimulation.setInputVoltage(voltage);
	}

	@Override
	public void updateMotor() {
		elevatorSimulation.update(TimeUtil.getLatestCycleTimeSeconds());
	}

	@Override
	public double getGearRatio() {
		return gearRatio;
	}

}
