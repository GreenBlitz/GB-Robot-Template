package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Robot;
import frc.robot.hardware.motor.IMotor;
import frc.robot.hardware.signal.DoubleSignal;
import frc.robot.hardware.signal.InputSignal;
import frc.robot.hardware.signal.TimedValue;
import frc.robot.hardware.signal.phoenix.Phoenix6DoubleSignal;

public class Factory {

    public static Intake create(){
        IntakeStuff intakeStuff;
        switch (Robot.ROBOT_TYPE){
            case REAL:

            case SIMULATION:
//                motor = Constants.
//                signal =
        }
//        return new Intake(Constants.LOG_PATH, motor, digitalInput, signal);
    }

}
