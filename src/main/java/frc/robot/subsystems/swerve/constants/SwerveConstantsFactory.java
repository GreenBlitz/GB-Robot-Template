package frc.robot.subsystems.swerve.constants;

import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveConstants;

public class SwerveConstantsFactory {

    public static SwerveConstants createSwerveConstants(){
        return switch (Robot.ROBOT_TYPE) {
            case REAL, REPLAY -> SwerveConstantsContainer.mk4iSwerveConstants;
            case SIMULATION -> SwerveConstantsContainer.simulationSwerveConstants;
        };
    }

}
