package frc.utils;

import frc.robot.Robot;

public class RobotTypeUtils {

    public static RobotType determineRobotType(RobotType wantedType) {
        return Robot.isSimulation() ? (wantedType.isReplay() ? RobotType.REPLAY : RobotType.SIMULATION) : RobotType.REAL;
    }

    public enum RobotType {

        REAL,
        SIMULATION,
        REPLAY;

        public boolean isReal() {
            return this.equals(RobotType.REAL);
        }

        public boolean isSimulation() {
            return this.equals(RobotType.SIMULATION);
        }

        public boolean isReplay() {
            return this.equals(RobotType.REPLAY);
        }
    }

}
