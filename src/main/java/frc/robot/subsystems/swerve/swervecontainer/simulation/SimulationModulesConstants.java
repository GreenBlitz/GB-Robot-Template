package frc.robot.subsystems.swerve.swervecontainer.simulation;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.swerve.modules.simulation.SimulationModuleConstants;

public class SimulationModulesConstants {

    private static final double WHEEL_DIAMETER_METERS = 0.048359 * 2;

    private static final boolean ENABLE_FOC_STEER = true;
    private static final boolean ENABLE_FOC_DRIVE = true;

    private static final PIDConstants STEER_PID_CONSTANTS = new PIDConstants(72, 0, 0);

    protected static SimulationModuleConstants getModuleConstants() {
        return new SimulationModuleConstants(
                WHEEL_DIAMETER_METERS,
                SimulationSwerve.VELOCITY_AT_12_VOLTS_METERS_PER_SECOND,
                new DCMotorSim(DCMotor.getFalcon500Foc(1), 150.0 / 7.0, 0.00001),
                new DCMotorSim(DCMotor.getFalcon500Foc(1), 6.12, 0.001),
                ENABLE_FOC_STEER,
                ENABLE_FOC_DRIVE,
                STEER_PID_CONSTANTS
        );
    }

}
