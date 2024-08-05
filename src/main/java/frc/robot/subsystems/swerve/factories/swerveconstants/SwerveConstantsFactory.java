package frc.robot.subsystems.swerve.factories.swerveconstants;

import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveName;

public class SwerveConstantsFactory {

    public static SwerveConstants create(SwerveName swerveName){
        return switch (Robot.ROBOT_TYPE) {
            case REAL, REPLAY -> getRealSwerveConstants(swerveName);
            case SIMULATION -> SimulationSwerveConstants.getSwerveConstants(swerveName);
        };
    }

    private static SwerveConstants getRealSwerveConstants(SwerveName swerveName) {
        return switch (swerveName) {
            case SWERVE -> RealSwerveConstants.swerveConstants;
        };
    }

}
