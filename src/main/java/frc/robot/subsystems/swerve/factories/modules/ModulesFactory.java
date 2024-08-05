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

    private static Module createModule(SwerveName swerveName, ModuleUtils.ModulePosition modulePosition) {
        return new Module(
                modulePosition,
                ModuleConstantsFactory.create(swerveName),
                EncoderFactory.create(modulePosition),
                SteerFactory.create(modulePosition, swerveName),
                DriveFactory.create(modulePosition, swerveName)
        );
    }

    public static Modules create(SwerveName swerveName) {
        return new Modules(
                swerveName,
                new Module[]{
                        createModule(swerveName, ModuleUtils.ModulePosition.FRONT_LEFT),
                        createModule(swerveName, ModuleUtils.ModulePosition.FRONT_RIGHT),
                        createModule(swerveName, ModuleUtils.ModulePosition.BACK_LEFT),
                        createModule(swerveName, ModuleUtils.ModulePosition.BACK_RIGHT)
                }
        );
    }

}
