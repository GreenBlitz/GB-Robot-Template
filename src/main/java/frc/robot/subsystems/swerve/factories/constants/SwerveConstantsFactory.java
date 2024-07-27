package frc.robot.subsystems.swerve.factories.constants;

import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveConstants;

public class SwerveConstantsFactory {

    public static SwerveConstants create(){
        return switch (Robot.ROBOT_TYPE) {
            case REAL, REPLAY -> RealSwerveConstants.swerveConstants;
            case SIMULATION -> SimulationSwerveConstants.swerveConstants;
        };
    }

}
