package frc.robot.subsystems.swerve.gyro;

import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.swerve.gyro.pigeon2swervegyro.Pigeon2SwerveGyro;

public class SwerveGyroFactory {

    public static ISwerveGyro createSwerve() {
        return switch (RobotConstants.ROBOT_TYPE) {
            case REAL -> new Pigeon2SwerveGyro();
            case SIMULATION -> new SimulationSwerveGyro();
            case REPLAY -> new ReplaySwerveGyro();
        };
    }

}
