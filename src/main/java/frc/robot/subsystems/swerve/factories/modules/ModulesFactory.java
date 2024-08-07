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
                ModuleConstantsFactory.create(swerveName, modulePosition),
                EncoderFactory.create(swerveName, modulePosition),
                SteerFactory.create(swerveName, modulePosition),
                DriveFactory.create(swerveName, modulePosition)
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
