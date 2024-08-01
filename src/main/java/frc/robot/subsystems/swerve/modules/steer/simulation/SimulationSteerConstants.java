package frc.robot.subsystems.swerve.modules.steer.simulation;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.simulation.SimpleMotorSimulation;

public class SimulationSteerConstants{

    private final SimpleMotorSimulation steerMotor;
    private final boolean enableFOC;

    public SimulationSteerConstants(DCMotorSim steerMotor, PIDConstants steerPIDConstants, boolean enableFOC){
        this.steerMotor = new SimpleMotorSimulation(steerMotor);
        this.enableFOC = enableFOC;

        TalonFXConfiguration steerConfiguration = new TalonFXConfiguration();
        steerConfiguration.Slot0.kP = steerPIDConstants.kP;
        steerConfiguration.Slot0.kI = steerPIDConstants.kI;
        steerConfiguration.Slot0.kD = steerPIDConstants.kD;
        steerConfiguration.ClosedLoopGeneral.ContinuousWrap = true;
        this.steerMotor.applyConfiguration(steerConfiguration);
    }

    protected SimpleMotorSimulation getSteerMotor() {
        return steerMotor;
    }

    protected boolean getEnableFOC(){
        return enableFOC;
    }

}
