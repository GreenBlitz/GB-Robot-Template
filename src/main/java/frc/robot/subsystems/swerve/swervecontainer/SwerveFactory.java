package frc.robot.subsystems.swerve.swervecontainer;

import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.gyro.ISwerveGyro;
import frc.robot.subsystems.swerve.gyro.replay.ReplaySwerveGyro;
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

    private static final RealSwerve REAL_SWERVE = new RealSwerve(new SwerveContainerKey());
    private static final SimulationSwerve SIMULATION_SWERVE = new SimulationSwerve(new SwerveContainerKey());


    public static SwerveConstants createConstants() {
        return switch (ROBOT_TYPE) {
            case REAL, REPLAY -> REAL_SWERVE.constants;
            case SIMULATION -> SIMULATION_SWERVE.constants;
        };
    }

    public static ISwerveGyro createGyro() {
        return switch (ROBOT_TYPE) {
            case REAL -> REAL_SWERVE.gyro;
            case SIMULATION -> SIMULATION_SWERVE.gyro;
            case REPLAY -> new ReplaySwerveGyro();
        };
    }

    public static Module[] createModules(){
        return switch (ROBOT_TYPE) {
            case REAL -> new Module[]{
                    new Module(ModuleUtils.ModuleName.FRONT_LEFT, REAL_SWERVE.modules[0]),
                    new Module(ModuleUtils.ModuleName.FRONT_RIGHT, REAL_SWERVE.modules[1]),
                    new Module(ModuleUtils.ModuleName.BACK_LEFT, REAL_SWERVE.modules[2]),
                    new Module(ModuleUtils.ModuleName.BACK_RIGHT, REAL_SWERVE.modules[3])
            };
            case SIMULATION -> new Module[]{
                    new Module(ModuleUtils.ModuleName.FRONT_LEFT, SIMULATION_SWERVE.modules[0]),
                    new Module(ModuleUtils.ModuleName.FRONT_RIGHT, SIMULATION_SWERVE.modules[1]),
                    new Module(ModuleUtils.ModuleName.BACK_LEFT, SIMULATION_SWERVE.modules[2]),
                    new Module(ModuleUtils.ModuleName.BACK_RIGHT, SIMULATION_SWERVE.modules[3])
            };
            case REPLAY -> new Module[]{
                    new Module(ModuleUtils.ModuleName.FRONT_LEFT, new ReplayModule()),
                    new Module(ModuleUtils.ModuleName.FRONT_RIGHT, new ReplayModule()),
                    new Module(ModuleUtils.ModuleName.BACK_LEFT, new ReplayModule()),
                    new Module(ModuleUtils.ModuleName.BACK_RIGHT, new ReplayModule())
            };
        };
    }

}
