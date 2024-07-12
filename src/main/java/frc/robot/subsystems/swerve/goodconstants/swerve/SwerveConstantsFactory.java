package frc.robot.subsystems.swerve.goodconstants.swerve;

import frc.robot.Robot;

public class SwerveConstantsFactory {

    public static SwerveConstantsObject createSwerveConstants(){
        return switch (Robot.ROBOT_TYPE) {
            case REAL, REPLAY -> SwerveConstantsContainer.mk4iSwerveConstants;
            case SIMULATION -> SwerveConstantsContainer.simulationSwerveConstants;
        };
    }

}
