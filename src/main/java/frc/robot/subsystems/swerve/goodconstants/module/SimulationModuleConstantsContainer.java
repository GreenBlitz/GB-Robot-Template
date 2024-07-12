package frc.robot.subsystems.swerve.goodconstants.module;


import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SimulationModuleConstantsContainer {

    public static SimulationModuleConstantsObject getSimulationModuleConstantsObject(double maxVelocityMetersPerSecond){
        return new SimulationModuleConstantsObject(
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
