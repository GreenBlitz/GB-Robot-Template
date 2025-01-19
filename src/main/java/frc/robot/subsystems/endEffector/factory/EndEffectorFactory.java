package frc.robot.subsystems.endEffector.factory;

import frc.robot.subsystems.endEffector.EndEffector;
import frc.robot.subsystems.endEffector.EndEffectorConstants;

public class EndEffectorFactory {

    public static EndEffector create(){
        return EndEffectorSparkMaxBuilder.generate(EndEffectorConstants.LOG_PATH, EndEffectorConstants.MOTOR_LOG_PATH);
    }

}
