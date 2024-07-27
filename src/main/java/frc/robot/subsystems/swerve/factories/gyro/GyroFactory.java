package frc.robot.subsystems.swerve.factories.gyro;

import frc.robot.Robot;
import frc.robot.subsystems.swerve.gyro.ISwerveGyro;
import frc.robot.subsystems.swerve.gyro.pigeon2.Pigeon2Gyro;
import frc.robot.subsystems.swerve.gyro.replay.ReplaySwerveGyro;

public class GyroFactory {

    public static ISwerveGyro create(){
        return switch (Robot.ROBOT_TYPE) {
            case REAL -> new Pigeon2Gyro(RealGyroConstants.PIGEON_2_GYRO_CONFIG_OBJECT);
            case SIMULATION, REPLAY -> new ReplaySwerveGyro();
        };
    }

}
