package frc.robot.subsystems.swerve.gyro;

import frc.robot.constants.Ports;
import frc.robot.subsystems.swerve.gyro.pigeon2.Pigeon2Gyro;
import frc.robot.subsystems.swerve.gyro.replay.ReplaySwerveGyro;
import frc.robot.subsystems.swerve.gyro.simulation.SimulationSwerveGyro;
import frc.utils.RobotTypeUtils.RobotType;

public class SwerveGyroFactory {

    public static ISwerveGyro createSwerveGyro(RobotType robotType) {
        return switch (robotType) {
            case REAL -> new Pigeon2Gyro(Ports.PIGEON_2_DEVICE_ID);
            case SIMULATION -> new SimulationSwerveGyro();
            case REPLAY -> new ReplaySwerveGyro();
        };
    }

}
