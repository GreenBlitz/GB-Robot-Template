package frc.robot.simulation;

import frc.robot.hardware.motor.phoenix6.TalonFXWrapper;

import java.util.ArrayList;
import java.util.List;

public class SimulationManager {

	private static final List<MotorSimulation> REGISTERED_SIMULATIONS = new ArrayList<>();

	static TalonFXWrapper createNewMotorForSimulation() {
		return new TalonFXWrapper(REGISTERED_SIMULATIONS.size());
	}

	static void addSimulation(MotorSimulation simulation) {
		REGISTERED_SIMULATIONS.add(simulation);
	}

	public static void updateRegisteredSimulations() {
		for (MotorSimulation motorSimulation : REGISTERED_SIMULATIONS) {
			motorSimulation.updateSimulation();
		}
	}

}
