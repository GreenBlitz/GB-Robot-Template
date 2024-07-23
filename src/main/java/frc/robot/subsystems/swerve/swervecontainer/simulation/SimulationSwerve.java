package frc.robot.subsystems.swerve.swervecontainer.simulation;

import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.gyro.simulation.SimulationSwerveGyro;
import frc.robot.subsystems.swerve.modules.ModuleUtils;
import frc.robot.subsystems.swerve.modules.simulation.SimulationModule;
import frc.robot.subsystems.swerve.swervecontainer.SwerveContainer;
import frc.robot.subsystems.swerve.swervecontainer.SwerveFactory;

public class SimulationSwerve extends SwerveContainer {

    private static SimulationSwerve instance;

    public static SimulationSwerve getInstance(SwerveFactory.SwerveContainerKey key) {
        if (instance == null){
            instance = new SimulationSwerve(key);
        }
        return instance;
    }

    protected static final double VELOCITY_AT_12_VOLTS_METERS_PER_SECOND = 5.052;

    private final SwerveConstants constants;
    private final SimulationSwerveGyro gyro;
    private final SimulationModule[] modules;

    private SimulationSwerve(SwerveFactory.SwerveContainerKey key){
        super(key);
        this.constants = SimulationSwerveConstants.swerveConstants;
        this.gyro = new SimulationSwerveGyro();
        this.modules = new SimulationModule[]{
                new SimulationModule(ModuleUtils.ModuleName.FRONT_LEFT, SimulationModulesConstants.getModuleConstants()),
                new SimulationModule(ModuleUtils.ModuleName.FRONT_RIGHT, SimulationModulesConstants.getModuleConstants()),
                new SimulationModule(ModuleUtils.ModuleName.BACK_LEFT, SimulationModulesConstants.getModuleConstants()),
                new SimulationModule(ModuleUtils.ModuleName.BACK_RIGHT, SimulationModulesConstants.getModuleConstants())
        };
    }

    @Override
    public SwerveConstants getConstants() {
        return constants;
    }

    @Override
    public SimulationSwerveGyro getGyro() {
        return gyro;
    }

    @Override
    public SimulationModule[] getModules() {
        return modules;
    }

}
