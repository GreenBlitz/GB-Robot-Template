package frc.robot.subsystems.swerve.modules.steer.simulation;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.simulation.SimpleMotorSimulation;

public class SimulationSteerConstants {

	private final SimpleMotorSimulation motor;
	private final boolean enableFOC;

	public SimulationSteerConstants(DCMotorSim dcMotorSim, TalonFXConfiguration configuration, boolean enableFOC) {
		this.motor = new SimpleMotorSimulation(dcMotorSim);
		this.enableFOC = enableFOC;
		motor.applyConfiguration(configuration);
	}

	protected SimpleMotorSimulation getMotor() {
		return motor;
	}

	protected boolean getEnableFOC() {
		return enableFOC;
	}

}
