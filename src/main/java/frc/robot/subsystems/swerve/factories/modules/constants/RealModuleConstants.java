package frc.robot.subsystems.swerve.factories.modules.constants;

import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.factories.swerveconstants.RealSwerveConstants;
import frc.robot.subsystems.swerve.modules.ModuleConstants;

public class RealModuleConstants {

    private static final double WHEEL_DIAMETER_METERS = 0.048359 * 2;
    private static final double COUPLING_RATIO = 0.59;
    private static final String MODULES_NAME = "Real";

    protected static ModuleConstants getModuleConstants(){
        return new ModuleConstants(
                SwerveConstants.getSwerveLogPath(RealSwerveConstants.SWERVE_NAME),
                MODULES_NAME,
                WHEEL_DIAMETER_METERS,
                COUPLING_RATIO,
                RealSwerveConstants.VELOCITY_AT_12_VOLTS_METERS_PER_SECOND
        );
    }

}
