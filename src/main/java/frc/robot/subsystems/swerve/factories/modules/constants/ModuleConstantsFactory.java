package frc.robot.subsystems.swerve.factories.modules.constants;

import frc.robot.Robot;
import frc.robot.subsystems.swerve.modules.ModuleConstants;

public class ModuleConstantsFactory {

    public static ModuleConstants create(){
        return switch (Robot.ROBOT_TYPE){
            case REAL, REPLAY -> RealModuleConstants.getModuleConstants();
            case SIMULATION -> SimulationModuleConstants.getModuleConstants();
        };
    }

}
