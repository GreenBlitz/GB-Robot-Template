package frc.robot.subsystems.swerve.gyro.gyrointerface;

import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.swerve.gyro.replay.ReplaySwerveGyro;
import frc.robot.subsystems.swerve.gyro.simulation.SimulationSwerveGyro;
import frc.robot.subsystems.swerve.gyro.pigeon2.Pigeon2Gyro;

public class SwerveGyroFactory {

    public static ISwerveGyro createSwerveGyro() {
        return switch (RobotConstants.ROBOT_TYPE) {
            case REAL -> new Pigeon2Gyro();
            case SIMULATION -> new SimulationSwerveGyro();
            case REPLAY -> new ReplaySwerveGyro();
        };
    }

}
