package frc.robot.subsystems.intake.factory;

import frc.robot.Robot;
import frc.robot.subsystems.intake.Intake;

public class Factory {

    public static Intake create(){
        return switch (Robot.ROBOT_TYPE){
            case REAL -> new Intake(RealConstants.generateIntake());
            case SIMULATION -> null;
        };
    }

}
