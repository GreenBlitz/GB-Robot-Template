package frc.robot.subsystems.swerve.swervecontainer.real;

import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.gyro.pigeon2.Pigeon2Gyro;
import frc.robot.subsystems.swerve.modules.talonfx.TalonFXModule;

public class RealSwerve {

    protected static final double VELOCITY_AT_12_VOLTS_METERS_PER_SECOND = 5.052;

    public static final Pigeon2Gyro gyro = new Pigeon2Gyro(GyroContainer.pigeon2GyroConfigObject);

    public static final TalonFXModule[] modules = {
            new TalonFXModule(ModulesContainer.moduleConstants[0]),
            new TalonFXModule(ModulesContainer.moduleConstants[1]),
            new TalonFXModule(ModulesContainer.moduleConstants[2]),
            new TalonFXModule(ModulesContainer.moduleConstants[3])
    };

    public static final SwerveConstants constants = ConstantsContainer.swerveConstants;


}
