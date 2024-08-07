package frc.robot.subsystems.swerve.modules.drive.simulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.simulation.SimpleMotorSimulation;

public record SimulationDriveConstants(
	SimpleMotorSimulation motorSimulation,
	boolean enableFOC,
	Rotation2d maxVelocityPerSecond
) {

	public SimulationDriveConstants(DCMotorSim dcMotorSim, boolean enableFOC, Rotation2d maxVelocityPerSecond) {
		this(new SimpleMotorSimulation(dcMotorSim), enableFOC, maxVelocityPerSecond);
	}

}
