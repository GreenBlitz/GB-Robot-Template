package frc.robot.subsystems.swerve.swerveinterface;

import frc.robot.subsystems.swerve.mk4iswerve.mk4imodules.MK4IModule;
import frc.robot.subsystems.swerve.replayswerve.ReplayModule;
import frc.robot.subsystems.swerve.simulationswerve.SimulationModule;
import frc.utils.RobotTypeUtils;

public class ModuleFactory {
    public enum ModuleName {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT;
    }

    public static IModule createModule(ModuleName moduleName){
        return switch (RobotTypeUtils.getRobotType()){
            case REAL -> new MK4IModule(moduleName);
            case SIMULATION -> new SimulationModule();
            case REPLAY -> new ReplayModule();
        };
    }

}
