package frc.robot.subsystems.swerve.typeconstants;

import frc.robot.Robot;

public class SwerveConstantsFactory {

    public static ISwerveConstants createSwerveConstants(){
        return switch (Robot.ROBOT_TYPE) {
            case REAL, REPLAY -> new RealSwerveConstants();
            case SIMULATION -> new SimulationSwerveConstants();
        };
    }

}
