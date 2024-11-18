package frc.robot.hardware.rev.simulation;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.hardware.rev.motors.SparkMaxDeviceID;
import frc.robot.hardware.rev.motors.SparkMaxWrapper;
import frc.utils.Conversions;

public class RevSimulationManager {

    private final REVPhysicsSim revPhysicsSim;

    public RevSimulationManager(){
        this.revPhysicsSim = new REVPhysicsSim();
    }

    //Maybe do this always? is sparkmax always neo for us? does this even work?
    public void addSparkMax(SparkMaxWrapper sparkMaxWrapper){
        revPhysicsSim.addSparkMax(sparkMaxWrapper, DCMotor.getNEO(1));
    }

    public void addSparkMax(SparkMaxWrapper sparkMax, DCMotor dcMotor){
        revPhysicsSim.addSparkMax(sparkMax, dcMotor);
    }

    public void addSparkMax(SparkMaxWrapper sparkMax, final float stallTorque, final float maxFreeSpeed){
        revPhysicsSim.addSparkMax(sparkMax, stallTorque, maxFreeSpeed);
    }

    public void run(){
        revPhysicsSim.run();
    }

}
