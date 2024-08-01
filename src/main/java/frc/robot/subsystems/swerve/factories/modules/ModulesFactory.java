package frc.robot.subsystems.swerve.factories.modules;

import frc.robot.subsystems.swerve.factories.modules.constants.ModuleConstantsFactory;
import frc.robot.subsystems.swerve.factories.modules.drive.DriveFactory;
import frc.robot.subsystems.swerve.factories.modules.encoder.EncoderFactory;
import frc.robot.subsystems.swerve.factories.modules.steer.SteerFactory;
import frc.robot.subsystems.swerve.modules.Module;
import frc.robot.subsystems.swerve.modules.ModuleConstants;
import frc.robot.subsystems.swerve.modules.ModuleUtils;
import frc.robot.subsystems.swerve.modules.Modules;
import frc.robot.subsystems.swerve.modules.drive.IDrive;
import frc.robot.subsystems.swerve.modules.encoder.IEncoder;
import frc.robot.subsystems.swerve.modules.steer.ISteer;


public class ModulesFactory {

    public static Modules create() {
        ISteer[] iSteers = SteerFactory.create();
        IDrive[] iDrives = DriveFactory.create();
        IEncoder[] iEncoders = EncoderFactory.create();
        ModuleConstants constants = ModuleConstantsFactory.create();

        return new Modules(new Module[]{
                new Module(ModuleUtils.ModuleName.FRONT_LEFT, iSteers[0], iDrives[0], iEncoders[0], constants),
                new Module(ModuleUtils.ModuleName.FRONT_LEFT, iSteers[1], iDrives[1], iEncoders[1], constants),
                new Module(ModuleUtils.ModuleName.FRONT_LEFT, iSteers[2], iDrives[2], iEncoders[2], constants),
                new Module(ModuleUtils.ModuleName.FRONT_LEFT, iSteers[3], iDrives[3], iEncoders[3], constants)
        });
    }

}
