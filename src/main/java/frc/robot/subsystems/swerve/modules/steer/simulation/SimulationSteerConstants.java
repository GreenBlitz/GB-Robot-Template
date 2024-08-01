package frc.robot.subsystems.swerve.modules.steer.simulation;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.simulation.SimpleMotorSimulation;

public record SimulationSteerConstants(SimpleMotorSimulation steerMotor, TalonFXConfiguration steerConfig, boolean enableFOC) {

    public SimulationSteerConstants(DCMotorSim steerMotor, TalonFXConfiguration steerConfig, boolean enableFOC){
        this(new SimpleMotorSimulation(steerMotor), steerConfig, enableFOC);
    }

}
