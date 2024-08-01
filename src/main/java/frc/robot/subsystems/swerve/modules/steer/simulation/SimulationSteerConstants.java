package frc.robot.subsystems.swerve.modules.steer.simulation;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.simulation.SimpleMotorSimulation;

public class SimulationSteerConstants{

    private final SimpleMotorSimulation steerMotor;
    private final boolean enableFOC;

    public SimulationSteerConstants(DCMotorSim steerDCMotor, TalonFXConfiguration steerConfiguration, boolean enableFOC){
        this.steerMotor = new SimpleMotorSimulation(steerDCMotor);
        this.enableFOC = enableFOC;
        steerMotor.applyConfiguration(steerConfiguration);
    }

    protected SimpleMotorSimulation getSteerMotor() {
        return steerMotor;
    }

    protected boolean getEnableFOC(){
        return enableFOC;
    }

}
