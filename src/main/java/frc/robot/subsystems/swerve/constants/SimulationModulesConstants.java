package frc.robot.subsystems.swerve.constants;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.swerve.modules.simulation.SimulationModuleConstants;

class SimulationModulesConstants {

    private static final double WHEEL_DIAMETER_METERS = 0.048359 * 2;

    private static final double STEER_GEAR_RATIO = 150.0 / 7.0;
    private static final double DRIVE_GEAR_RATIO = 6.12;

    private static final double STEER_MOMENT_OF_INERTIA = 0.00001;
    private static final double DRIVE_MOMENT_OF_INERTIA = 0.001;

    private static final boolean ENABLE_FOC_STEER = true;
    private static final boolean ENABLE_FOC_DRIVE = true;

    private static final PIDConstants STEER_PID_CONSTANTS = new PIDConstants(72, 0, 0);

    protected static SimulationModuleConstants getModuleConstants(double velocityAt12VoltsMetersPerSecond) {
        return new SimulationModuleConstants(
                WHEEL_DIAMETER_METERS,
                velocityAt12VoltsMetersPerSecond,
                new DCMotorSim(DCMotor.getFalcon500Foc(1), STEER_GEAR_RATIO, STEER_MOMENT_OF_INERTIA),
                new DCMotorSim(DCMotor.getFalcon500Foc(1), DRIVE_GEAR_RATIO, DRIVE_MOMENT_OF_INERTIA),
                ENABLE_FOC_STEER,
                ENABLE_FOC_DRIVE,
                STEER_PID_CONSTANTS
        );
    }

}
