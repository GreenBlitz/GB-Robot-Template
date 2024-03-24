package frc.robot.subsystems.swerve.swerveinterface;

import frc.robot.subsystems.swerve.mk4iswerve.mk4imodules.MK4IModule;
import frc.robot.subsystems.swerve.mk4iswerve.mk4imodules.MK4IModuleConstants;
import frc.robot.subsystems.swerve.replayswerve.ReplayModule;
import frc.robot.subsystems.swerve.simulationswerve.SimulationModule;
import frc.robot.subsystems.swerve.swerveinterface.IModule;
import frc.utils.RobotTypeUtils;

public class ModuleFactory {
    public enum Module {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT;
    }

    public static IModule generateModule(Module module){
        return switch (module){
            case FRONT_LEFT -> createFrontLeftModule();
            case FRONT_RIGHT -> createFrontRightModule();
            case BACK_LEFT -> createBackLeftModule();
            case BACK_RIGHT -> createBackRightModule();
        };
    }

    public static IModule createFrontLeftModule(){
        return switch (RobotTypeUtils.getRobotType()){
            case REAL -> new MK4IModule(MK4IModuleConstants.FRONT_LEFT);
            case SIMULATION -> new SimulationModule();
            case REPLAY -> new ReplayModule();
        };
    }

    public static IModule createFrontRightModule(){
        return switch (RobotTypeUtils.getRobotType()){
            case REAL -> new MK4IModule(MK4IModuleConstants.FRONT_RIGHT);
            case SIMULATION -> new SimulationModule();
            case REPLAY -> new ReplayModule();
        };
    }

    public static IModule createBackLeftModule(){
        return switch (RobotTypeUtils.getRobotType()){
            case REAL -> new MK4IModule(MK4IModuleConstants.BACK_LEFT);
            case SIMULATION -> new SimulationModule();
            case REPLAY -> new ReplayModule();
        };
    }

    public static IModule createBackRightModule(){
        return switch (RobotTypeUtils.getRobotType()){
            case REAL -> new MK4IModule(MK4IModuleConstants.BACK_RIGHT);
            case SIMULATION -> new SimulationModule();
            case REPLAY -> new ReplayModule();
        };
    }
}
