package frc.robot.subsystems.swerve.swervecontainer;

import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.gyro.ISwerveGyro;
import frc.robot.subsystems.swerve.modules.IModule;

public abstract class SwerveContainer {

    public SwerveContainer(SwerveFactory.SwerveContainerKey key){}

    public abstract SwerveConstants createConstants();
    public abstract ISwerveGyro createGyro();
    public abstract IModule[] createModules();

}
