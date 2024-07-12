package frc.robot.subsystems.swerve.modules.simulation;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.robot.simulation.SimpleMotorSimulation;

public class SimulationModuleConfigObject {

    private final SimpleMotorSimulation driveMotor, steerMotor;

    public SimulationModuleConfigObject(
            SimpleMotorSimulation steerMotor,
            SimpleMotorSimulation driveMotor,
            TalonFXConfiguration steerConfig
    ) {
        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;

        configureDriveMotor();
        configureSteerMotor(steerConfig);
    }

    private void configureDriveMotor() {
        driveMotor.applyConfiguration(new TalonFXConfiguration());
    }
    private void configureSteerMotor(TalonFXConfiguration steerConfig) {
        steerMotor.applyConfiguration(steerConfig);
    }


    public SimpleMotorSimulation getDriveMotor() {
        return driveMotor;
    }
    public SimpleMotorSimulation getSteerMotor() {
        return steerMotor;
    }

}
