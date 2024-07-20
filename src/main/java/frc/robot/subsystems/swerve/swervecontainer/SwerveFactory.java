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

    public static SwerveConstants createConstants() {
        return switch (ROBOT_TYPE) {
            case REAL, REPLAY -> RealSwerve.constants;
            case SIMULATION -> SimulationSwerve.constants;
        };
    }

    public static ISwerveGyro createGyro() {
        return switch (ROBOT_TYPE) {
            case REAL -> RealSwerve.gyro;
            case SIMULATION -> SimulationSwerve.gyro;
            case REPLAY -> new ReplaySwerveGyro();
        };
    }

    public static Module[] createModules(){
        return switch (ROBOT_TYPE) {
            case REAL -> new Module[]{
                    new Module(ModuleUtils.ModuleName.FRONT_LEFT, RealSwerve.modules[0]),
                    new Module(ModuleUtils.ModuleName.FRONT_RIGHT, RealSwerve.modules[1]),
                    new Module(ModuleUtils.ModuleName.BACK_LEFT, RealSwerve.modules[2]),
                    new Module(ModuleUtils.ModuleName.BACK_RIGHT, RealSwerve.modules[3])
            };
            case SIMULATION -> new Module[]{
                    new Module(ModuleUtils.ModuleName.FRONT_LEFT, SimulationSwerve.modules[0]),
                    new Module(ModuleUtils.ModuleName.FRONT_RIGHT, SimulationSwerve.modules[1]),
                    new Module(ModuleUtils.ModuleName.BACK_LEFT, SimulationSwerve.modules[2]),
                    new Module(ModuleUtils.ModuleName.BACK_RIGHT, SimulationSwerve.modules[3])
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
