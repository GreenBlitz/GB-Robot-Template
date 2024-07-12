package frc.robot.subsystems.swerve.modules.factory;


import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.swerve.modules.simulation.SimulationModuleConstants;

public class SimulationModuleContainer {

    public static SimulationModuleConstants simulationModuleConstants(double maxVelocityMetersPerSecond){
        return new SimulationModuleConstants(
                0.048359 * 2,
                maxVelocityMetersPerSecond,
                new DCMotorSim(DCMotor.getFalcon500Foc(1), 150.0 / 7.0, 0.00001),
                new DCMotorSim(DCMotor.getFalcon500Foc(1), 6.12, 0.001),
                true,
                true,
                new PIDConstants(72, 0, 0)
        );
    }


}
