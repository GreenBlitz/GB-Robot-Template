package frc.utils;

import frc.robot.Robot;

public class RobotTypeUtils {

    public enum RobotType {
        REAL,
        SIMULATION,
        REPLAY;

        public boolean isReal(){
            return this.equals(RobotType.REAL);
        }
        public boolean isSimulation(){
            return this.equals(RobotType.SIMULATION);
        }
        public boolean isReplay(){
            return this.equals(RobotType.REPLAY);
        }
    }


    public static RobotType determineRobotType(RobotType wantedType) {
        if (Robot.isSimulation()) {
            if (wantedType.equals(RobotType.REPLAY)) {
                return RobotType.REPLAY;
            }
            return RobotType.SIMULATION;
        } else {
            if (wantedType.equals(RobotType.REAL)) {
                return RobotType.REAL;
            }
        }
        return RobotType.REAL;
    }

}
