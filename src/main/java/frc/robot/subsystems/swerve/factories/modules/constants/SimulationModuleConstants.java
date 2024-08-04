package frc.robot.subsystems.swerve.factories.modules.constants;

import frc.robot.Robot;
import frc.robot.subsystems.swerve.factories.swerveconstants.SimulationSwerveConstants;
import frc.robot.subsystems.swerve.modules.ModuleConstants;

public class SimulationModuleConstants {

    public static final double WHEEL_DIAMETER_METERS = 0.048359 * 2;
    private static final double COUPLING_RATIO = 0;
    private static final String MODULES_NAME = "Simulation";

    protected static ModuleConstants getModuleConstants(){
        return new ModuleConstants(
                Robot.swerve.getLogPath(),
                MODULES_NAME,
                WHEEL_DIAMETER_METERS,
                COUPLING_RATIO,
                SimulationSwerveConstants.VELOCITY_AT_12_VOLTS_METERS_PER_SECOND
        );
    }

}
