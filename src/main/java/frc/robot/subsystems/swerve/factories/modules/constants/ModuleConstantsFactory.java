package frc.robot.subsystems.swerve.factories.modules.constants;

import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveName;
import frc.robot.subsystems.swerve.modules.ModuleConstants;

public class ModuleConstantsFactory {

    public static ModuleConstants create(SwerveName swerveName) {
        return switch (swerveName) {
            case SWERVE -> getSwerveModuleConstants(swerveName);
        };
    }

    private static ModuleConstants getSwerveModuleConstants(SwerveName swerveName) {
        return switch (Robot.ROBOT_TYPE) {
            case REAL, REPLAY -> RealModuleConstants.getModuleConstants(swerveName);
            case SIMULATION -> SimulationModuleConstants.getModuleConstants(swerveName);
        };
    }

}
