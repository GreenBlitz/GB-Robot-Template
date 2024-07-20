package frc.robot.subsystems.swerve.swervecontainer.simulation;

import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.gyro.simulation.SimulationSwerveGyro;
import frc.robot.subsystems.swerve.modules.ModuleUtils;
import frc.robot.subsystems.swerve.modules.simulation.SimulationModule;

public class SimulationSwerve {

    protected static final double VELOCITY_AT_12_VOLTS_METERS_PER_SECOND = 5.052;

    public static final SimulationSwerveGyro gyro = new SimulationSwerveGyro();

    public static final SimulationModule[] modules = {
            new SimulationModule(ModuleUtils.ModuleName.FRONT_LEFT, SimulationModulesConstants.getModuleConstants()),
            new SimulationModule(ModuleUtils.ModuleName.FRONT_RIGHT, SimulationModulesConstants.getModuleConstants()),
            new SimulationModule(ModuleUtils.ModuleName.BACK_LEFT, SimulationModulesConstants.getModuleConstants()),
            new SimulationModule(ModuleUtils.ModuleName.BACK_RIGHT, SimulationModulesConstants.getModuleConstants())
    };

    public static final SwerveConstants constants = SimulationSwerveConstants.swerveConstants;


}
