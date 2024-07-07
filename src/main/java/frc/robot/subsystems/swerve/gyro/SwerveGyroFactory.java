package frc.robot.subsystems.swerve.gyro;

import frc.robot.subsystems.swerve.gyro.pigeon2.Pigeon2Gyro;
import frc.robot.subsystems.swerve.gyro.pigeon2.Pigeon2GyroConstants;
import frc.robot.subsystems.swerve.gyro.replay.ReplaySwerveGyro;
import frc.robot.subsystems.swerve.gyro.simulation.SimulationSwerveGyro;
import frc.utils.RobotTypeUtils.RobotType;

public class SwerveGyroFactory {

    public static ISwerveGyro createSwerveGyro(RobotType robotType) {
        return switch (robotType) {
            case REAL -> new Pigeon2Gyro(Pigeon2GyroConstants.PIGEON_2_GYRO_CONFIG_OBJECT);
            case SIMULATION -> new SimulationSwerveGyro();
            case REPLAY -> new ReplaySwerveGyro();
        };
    }

}
