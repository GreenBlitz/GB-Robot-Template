package frc.robot.subsystems.swerve.factories.modules;

import frc.robot.subsystems.swerve.factories.modules.constants.ModuleConstantsFactory;
import frc.robot.subsystems.swerve.factories.modules.drive.DriveFactory;
import frc.robot.subsystems.swerve.factories.modules.encoder.EncoderFactory;
import frc.robot.subsystems.swerve.factories.modules.steer.SteerFactory;
import frc.robot.subsystems.swerve.modules.Module;
import frc.robot.subsystems.swerve.modules.ModuleUtils;
import frc.robot.subsystems.swerve.modules.Modules;


public class ModulesFactory {

    private static Module createModule(ModuleUtils.ModuleName moduleName) {
        return new Module(
                moduleName,
                ModuleConstantsFactory.create(),
                EncoderFactory.create(moduleName),
                SteerFactory.create(moduleName),
                DriveFactory.create(moduleName)
        );
    }

    public static Modules create() {
        return new Modules(new Module[]{
                createModule(ModuleUtils.ModuleName.FRONT_LEFT),
                createModule(ModuleUtils.ModuleName.FRONT_RIGHT),
                createModule(ModuleUtils.ModuleName.BACK_LEFT),
                createModule(ModuleUtils.ModuleName.BACK_RIGHT)
        });
    }

}
