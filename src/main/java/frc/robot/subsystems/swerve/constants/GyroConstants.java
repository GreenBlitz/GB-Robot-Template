package frc.robot.subsystems.swerve.constants;

import frc.robot.Robot;
import frc.robot.subsystems.swerve.gyro.ISwerveGyro;
import frc.robot.subsystems.swerve.gyro.SwerveGyroConstants;
import frc.robot.subsystems.swerve.gyro.pigeon2.Pigeon2Gyro;
import frc.robot.subsystems.swerve.gyro.pigeon2.Pigeon2GyroConfigObject;
import frc.robot.subsystems.swerve.gyro.replay.ReplaySwerveGyro;
import frc.robot.subsystems.swerve.gyro.simulation.SimulationSwerveGyro;

public class GyroConstants {

    public static ISwerveGyro create(SwerveGyroConstants constants){
        return switch (Robot.ROBOT_TYPE){
            case REAL -> new Pigeon2Gyro((Pigeon2GyroConfigObject) constants);
            case SIMULATION -> new SimulationSwerveGyro();
            case REPLAY -> new ReplaySwerveGyro();
        };
    }

}
