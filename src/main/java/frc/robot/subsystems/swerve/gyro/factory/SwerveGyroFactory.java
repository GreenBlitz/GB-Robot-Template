package frc.robot.subsystems.swerve.gyro.factory;

import frc.robot.Robot;
import frc.robot.subsystems.swerve.gyro.ISwerveGyro;
import frc.robot.subsystems.swerve.gyro.pigeon2.Pigeon2Gyro;
import frc.robot.subsystems.swerve.gyro.replay.ReplaySwerveGyro;
import frc.robot.subsystems.swerve.gyro.simulation.SimulationSwerveGyro;

public class SwerveGyroFactory {

    public static ISwerveGyro createSwerveGyro() {
        return switch (Robot.ROBOT_TYPE) {
            case REAL -> new Pigeon2Gyro(Pigeon2GyroContainer.PIGEON_2_GYRO_CONFIG_OBJECT);
            case SIMULATION -> new SimulationSwerveGyro();
            case REPLAY -> new ReplaySwerveGyro();
        };
    }

}
