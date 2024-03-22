package frc.utils;

import frc.robot.Robot;
import frc.robot.constants.RobotConstants;

public class RobotTypeUtils {

    public enum RobotType {
        REAL,
        SIMULATION,
        REPLAY
    }

    private static RobotType ROBOT_TYPE = determineRobotType();

    private static RobotType determineRobotType() {
        RobotType robotType = RobotConstants.ROBOT_TYPE;
        if (Robot.isSimulation()) {
            if (robotType.equals(RobotType.REPLAY)) {
                return RobotType.REPLAY;
            }
            return RobotType.SIMULATION;
        } else {
            if (robotType.equals(RobotType.REAL)) {
                return RobotType.REAL;
            }
        }
        return RobotType.REAL;
    }

    public static RobotType getRobotType(){
        return ROBOT_TYPE;
    }

    public static boolean isSimulation(){
        return ROBOT_TYPE.equals(RobotType.SIMULATION);
    }

    public static boolean isReal(){
        return ROBOT_TYPE.equals(RobotType.REAL);
    }

    public static boolean isReplay(){
        return ROBOT_TYPE.equals(RobotType.REPLAY);
    }

}
