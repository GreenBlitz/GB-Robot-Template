package frc.robot.subsystems.swerve.swervecontainer;

import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.gyro.ISwerveGyro;
import frc.robot.subsystems.swerve.gyro.pigeon2.Pigeon2Gyro;
import frc.robot.subsystems.swerve.gyro.replay.ReplaySwerveGyro;
import frc.robot.subsystems.swerve.modules.Module;
import frc.robot.subsystems.swerve.modules.ModuleUtils;
import frc.robot.subsystems.swerve.modules.replay.ReplayModule;
import frc.robot.subsystems.swerve.modules.simulation.SimulationModule;
import frc.robot.subsystems.swerve.modules.talonfx.TalonFXModule;

import static frc.robot.Robot.ROBOT_TYPE;

public class SwerveFactory {

    public static SwerveConstants createConstants() {
        return switch (ROBOT_TYPE) {
            case REAL, REPLAY -> RealSwerveContainer.swerveConstants;
            case SIMULATION -> SimulationSwerveContainer.swerveConstants;
        };
    }

    public static ISwerveGyro createGyro() {
        return switch (ROBOT_TYPE) {
            case REAL -> new Pigeon2Gyro(RealSwerveContainer.pigeon2GyroConfigObject);
            case REPLAY, SIMULATION -> new ReplaySwerveGyro();
        };
    }

    public static Module[] createModules(){
        return switch (ROBOT_TYPE) {
            case REAL -> new Module[]{
                    new Module(ModuleUtils.ModuleName.FRONT_LEFT, new TalonFXModule(RealSwerveContainer.moduleConstants[0])),
                    new Module(ModuleUtils.ModuleName.FRONT_RIGHT, new TalonFXModule(RealSwerveContainer.moduleConstants[1])),
                    new Module(ModuleUtils.ModuleName.BACK_LEFT, new TalonFXModule(RealSwerveContainer.moduleConstants[2])),
                    new Module(ModuleUtils.ModuleName.BACK_RIGHT, new TalonFXModule(RealSwerveContainer.moduleConstants[3]))
            };
            case SIMULATION -> new Module[]{
                    new Module(ModuleUtils.ModuleName.FRONT_LEFT, new SimulationModule(ModuleUtils.ModuleName.FRONT_LEFT, SimulationSwerveContainer.getModuleConstants())),
                    new Module(ModuleUtils.ModuleName.FRONT_RIGHT, new SimulationModule(ModuleUtils.ModuleName.FRONT_RIGHT, SimulationSwerveContainer.getModuleConstants())),
                    new Module(ModuleUtils.ModuleName.BACK_LEFT, new SimulationModule(ModuleUtils.ModuleName.BACK_LEFT, SimulationSwerveContainer.getModuleConstants())),
                    new Module(ModuleUtils.ModuleName.BACK_RIGHT, new SimulationModule(ModuleUtils.ModuleName.BACK_RIGHT, SimulationSwerveContainer.getModuleConstants()))
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
