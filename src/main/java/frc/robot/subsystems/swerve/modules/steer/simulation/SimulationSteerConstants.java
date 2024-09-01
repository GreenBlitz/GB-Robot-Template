package frc.robot.subsystems.swerve.modules.steer.simulation;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.simulation.SimpleMotorSimulation;
import frc.utils.calibration.sysid.SysIdCalibrator;

public class SimulationSteerConstants {

	private final SimpleMotorSimulation motor;
	private final boolean enableFOC;
	private final SysIdCalibrator.SysIdConfigInfo sysIdConfigInfo;

	public SimulationSteerConstants(
		DCMotorSim dcMotorSim,
		TalonFXConfiguration configuration,
		boolean enableFOC,
		SysIdRoutine.Config sysIdConfig
	) {
		this.motor = new SimpleMotorSimulation(dcMotorSim);
		this.enableFOC = enableFOC;
		this.sysIdConfigInfo = new SysIdCalibrator.SysIdConfigInfo(sysIdConfig, false);
		motor.applyConfiguration(configuration);
	}

	protected SimpleMotorSimulation getMotor() {
		return motor;
	}

	protected boolean getEnableFOC() {
		return enableFOC;
	}

	protected SysIdCalibrator.SysIdConfigInfo getSysIdConfigInfo() {
		return sysIdConfigInfo;
	}

}
