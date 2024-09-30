package frc.robot.subsystems.lifter.factory;

import frc.robot.Robot;
import frc.robot.subsystems.lifter.LifterStuff;

public class LifterFactory {

    public static LifterStuff create(String logPath){
        return switch (Robot.ROBOT_TYPE){
            case REAL -> LifterRealConstants.generateLifterStuff(logPath);
            case SIMULATION -> null;
        };
    }

}
