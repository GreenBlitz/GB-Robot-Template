package frc.robot.subsystems.swerve.gyro;

import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.swerve.gyro.gyropigeon2.GyroPigeon2;

public class GyroFactory {

    public static IGyro createSwerve() {
        return switch (RobotConstants.ROBOT_TYPE) {
            case REAL -> new GyroPigeon2();
            case SIMULATION -> new SimulationGyro();
            case REPLAY -> new ReplayGyro();
        };
    }

}
