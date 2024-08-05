package frc.robot.subsystems.swerve.factories.modules;

import frc.robot.subsystems.swerve.SwerveName;
import frc.robot.subsystems.swerve.factories.modules.constants.ModuleConstantsFactory;
import frc.robot.subsystems.swerve.factories.modules.drive.DriveFactory;
import frc.robot.subsystems.swerve.factories.modules.encoder.EncoderFactory;
import frc.robot.subsystems.swerve.factories.modules.steer.SteerFactory;
import frc.robot.subsystems.swerve.modules.Module;
import frc.robot.subsystems.swerve.modules.ModuleUtils;
import frc.robot.subsystems.swerve.modules.Modules;


public class ModulesFactory {

    private static Module createModule(SwerveName swerveName, ModuleUtils.ModulePosition modulePosition,
            ModuleUtils.ModuleType moduleType) {
        return new Module(
                modulePosition,
                ModuleConstantsFactory.create(swerveName, moduleType),
                EncoderFactory.create(modulePosition),
                SteerFactory.create(modulePosition, moduleType),
                DriveFactory.create(modulePosition, moduleType)
        );
    }

    public static Modules create(SwerveName swerveName, ModuleUtils.ModuleType moduleType) {
        return new Modules(
                swerveName,
                moduleType,
                new Module[]{
                        createModule(swerveName, ModuleUtils.ModulePosition.FRONT_LEFT, moduleType),
                        createModule(swerveName, ModuleUtils.ModulePosition.FRONT_RIGHT, moduleType),
                        createModule(swerveName, ModuleUtils.ModulePosition.BACK_LEFT, moduleType),
                        createModule(swerveName, ModuleUtils.ModulePosition.BACK_RIGHT, moduleType)
                }
        );
    }

}
