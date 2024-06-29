package frc.robot.simulation;

import frc.utils.devicewrappers.TalonFXWrapper;

import java.util.ArrayList;
import java.util.List;

public class SimulationManager {

    private static final List<MotorSimulation> REGISTERED_SIMULATIONS = new ArrayList<>();

    protected static void addSimulation(MotorSimulation simulation) {
        REGISTERED_SIMULATIONS.add(simulation);
    }

    protected static TalonFXWrapper createNewMotorForSimulation(){
        return new TalonFXWrapper(REGISTERED_SIMULATIONS.size());
    }

    public static void updateRegisteredSimulations() {
        for (MotorSimulation motorSimulation : REGISTERED_SIMULATIONS) {
            motorSimulation.updateSimulation();
        }
    }

}
