package frc.robot.subsystems.swerve.swervecontainer.real;

import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.gyro.pigeon2.Pigeon2Gyro;
import frc.robot.subsystems.swerve.modules.talonfx.TalonFXModule;
import frc.robot.subsystems.swerve.swervecontainer.SwerveContainer;
import frc.robot.subsystems.swerve.swervecontainer.SwerveFactory;

public class RealSwerve extends SwerveContainer {

    private static RealSwerve instance;

    public static RealSwerve getInstance(SwerveFactory.SwerveContainerKey key) {
        if (instance == null){
            instance = new RealSwerve(key);
        }
        return instance;
    }


    protected static final double VELOCITY_AT_12_VOLTS_METERS_PER_SECOND = 5.052;

    private RealSwerve(SwerveFactory.SwerveContainerKey key) {
        super(key);
    }

    @Override
    public SwerveConstants createConstants() {
        return RealSwerveConstants.swerveConstants;
    }

    @Override
    public Pigeon2Gyro createGyro() {
        return new Pigeon2Gyro(RealGyroConstants.pigeon2GyroConfigObject);
    }

    @Override
    public TalonFXModule[] createModules() {
        return new TalonFXModule[]{
                new TalonFXModule(RealModulesConstants.moduleConstants[0]),
                new TalonFXModule(RealModulesConstants.moduleConstants[1]),
                new TalonFXModule(RealModulesConstants.moduleConstants[2]),
                new TalonFXModule(RealModulesConstants.moduleConstants[3])
        };
    }

}
