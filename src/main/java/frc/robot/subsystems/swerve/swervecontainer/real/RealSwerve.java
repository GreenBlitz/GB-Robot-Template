package frc.robot.subsystems.swerve.swervecontainer.real;

import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.gyro.pigeon2.Pigeon2Gyro;
import frc.robot.subsystems.swerve.modules.talonfx.TalonFXModule;
import frc.robot.subsystems.swerve.swervecontainer.SwerveFactory;

public class RealSwerve {

    protected static final double VELOCITY_AT_12_VOLTS_METERS_PER_SECOND = 5.052;


    public final SwerveConstants constants;

    public final Pigeon2Gyro gyro;

    public final TalonFXModule[] modules;

    public RealSwerve(SwerveFactory.SwerveContainerKey key){
        this.constants = RealSwerveConstants.swerveConstants;
        this.gyro = new Pigeon2Gyro(RealGyroConstants.pigeon2GyroConfigObject);
        this.modules = new TalonFXModule[]{
                new TalonFXModule(RealModulesConstants.moduleConstants[0]),
                new TalonFXModule(RealModulesConstants.moduleConstants[1]),
                new TalonFXModule(RealModulesConstants.moduleConstants[2]),
                new TalonFXModule(RealModulesConstants.moduleConstants[3])
        };
    }

}
