package frc.robot.subsystems.swerve.swervecontainer;

import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.gyro.ISwerveGyro;
import frc.robot.subsystems.swerve.gyro.replay.ReplaySwerveGyro;
import frc.robot.subsystems.swerve.modules.IModule;
import frc.robot.subsystems.swerve.modules.Module;
import frc.robot.subsystems.swerve.modules.ModuleUtils;
import frc.robot.subsystems.swerve.modules.replay.ReplayModule;
import frc.robot.subsystems.swerve.swervecontainer.real.RealSwerve;
import frc.robot.subsystems.swerve.swervecontainer.simulation.SimulationSwerve;

import static frc.robot.Robot.ROBOT_TYPE;

public class SwerveFactory {

    public static class SwerveContainerKey {

        private SwerveContainerKey(){}

    }

    private static final SwerveContainer REAL_SWERVE = RealSwerve.getInstance(new SwerveContainerKey());
    private static final SwerveContainer SIMULATION_SWERVE = SimulationSwerve.getInstance(new SwerveContainerKey());


    public static SwerveConstants createConstants() {
        return switch (ROBOT_TYPE) {
            case REAL, REPLAY -> REAL_SWERVE.getConstants();
            case SIMULATION -> SIMULATION_SWERVE.getConstants();
        };
    }

    public static ISwerveGyro createGyro() {
        return switch (ROBOT_TYPE) {
            case REAL -> REAL_SWERVE.getGyro();
            case SIMULATION -> SIMULATION_SWERVE.getGyro();
            case REPLAY -> new ReplaySwerveGyro();
        };
    }

    private static Module[] getModules(IModule[] iModules){
        return new Module[]{
                new Module(ModuleUtils.ModuleName.FRONT_LEFT, iModules[0]),
                new Module(ModuleUtils.ModuleName.FRONT_RIGHT, iModules[1]),
                new Module(ModuleUtils.ModuleName.BACK_LEFT, iModules[2]),
                new Module(ModuleUtils.ModuleName.BACK_RIGHT, iModules[3])
        };
    }

    public static Module[] createModules(){
        return switch (ROBOT_TYPE) {
            case REAL -> getModules(REAL_SWERVE.getModules());
            case SIMULATION -> getModules(SIMULATION_SWERVE.getModules());
            case REPLAY -> getModules(new ReplayModule[]{
                    new ReplayModule(),
                    new ReplayModule(),
                    new ReplayModule(),
                    new ReplayModule()
            });
        };
    }

}
