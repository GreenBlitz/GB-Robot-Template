package frc.robot.subsystems.swerve.swervegyro.swervegyrointerface;

import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.swerve.swervegyro.ReplaySwerveGyro;
import frc.robot.subsystems.swerve.swervegyro.SimulationSwerveGyro;
import frc.robot.subsystems.swerve.swervegyro.pigeon2swervegyro.Pigeon2SwerveGyro;

public class SwerveGyroFactory {

    public static ISwerveGyro createSwerve() {
        return switch (RobotConstants.ROBOT_TYPE) {
            case REAL -> new Pigeon2SwerveGyro();
            case SIMULATION -> new SimulationSwerveGyro();
            case REPLAY -> new ReplaySwerveGyro();
        };
    }

}
