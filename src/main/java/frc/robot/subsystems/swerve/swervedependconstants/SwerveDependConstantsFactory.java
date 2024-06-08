package frc.robot.subsystems.swerve.swervedependconstants;

import frc.robot.constants.RobotConstants;

public class SwerveDependConstantsFactory {

    public static SwerveDependConstants createSwerveDependConstants() {
        return switch (RobotConstants.ROBOT_TYPE) {
            case REAL -> new SwerveRealConstants();
            case SIMULATION -> new SwerveSimulationConstants();
            case REPLAY -> new SwerveSimulationConstants();//todo - find what to do
        };
    }

}
