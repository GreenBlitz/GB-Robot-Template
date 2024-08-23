package frc.robot.subsystems.swerve.modules.drive.simulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.simulation.SimpleMotorSimulation;
import frc.utils.calibration.sysid.SysIdCalibrator;

public record SimulationDriveConstants(
	SimpleMotorSimulation motorSimulation,
	Rotation2d maxVelocityPerSecond,
	SysIdCalibrator.SysIdConfigInfo sysIdConfigInfo
) {

	public SimulationDriveConstants(DCMotorSim dcMotorSim, Rotation2d maxVelocityPerSecond, SysIdRoutine.Config sysIdConfig) {
		this(new SimpleMotorSimulation(dcMotorSim), maxVelocityPerSecond, new SysIdCalibrator.SysIdConfigInfo(sysIdConfig, false));
	}

}
