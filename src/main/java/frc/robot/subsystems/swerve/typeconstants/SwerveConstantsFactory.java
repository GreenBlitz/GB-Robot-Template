package frc.robot.subsystems.swerve.typeconstants;

import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveConstants;

public class SwerveConstantsFactory {

    public static SwerveConstants createSwerveConstants(){
        return switch (Robot.ROBOT_TYPE) {
            case REAL, REPLAY -> new RealSwerveConstants();
            case SIMULATION -> new SimulationSwerveConstants();
        };
    }

}
