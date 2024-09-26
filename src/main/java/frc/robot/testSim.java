package frc.robot;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.simulation.*;

public class testSim extends SingleJointedArmSimulation{

    public testSim(String logPath, SingleJointedArmSim armSimulation) {
        super(logPath, armSimulation);
    }

}
