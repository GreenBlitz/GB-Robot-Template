package frc.robot.subsystems.swerve.factories.modules.steer;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.swerve.modules.steer.simulation.SimulationSteerConstants;

public class SteerSimulationConstants {

    private static final double STEER_GEAR_RATIO = 150.0 / 7.0;

    private static final double STEER_MOMENT_OF_INERTIA = 0.00001;

    private static final boolean ENABLE_FOC_STEER = true;

    private static final PIDConstants STEER_PID_CONSTANTS = new PIDConstants(72, 0, 0);

    protected static SimulationSteerConstants getSteerConstants(){
        return new SimulationSteerConstants(
                new DCMotorSim(DCMotor.getFalcon500Foc(1), STEER_GEAR_RATIO, STEER_MOMENT_OF_INERTIA),
                STEER_PID_CONSTANTS,
                ENABLE_FOC_STEER
        );
    }
}
