package frc.robot.subsystems.swerve.modules.simulation;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.robot.simulation.SimpleMotorSimulation;

class SimulationModuleConfigObject {

    private final SimpleMotorSimulation steerMotor, driveMotor;

    protected SimulationModuleConfigObject(
            SimpleMotorSimulation steerMotor,
            SimpleMotorSimulation driveMotor,
            TalonFXConfiguration steerConfig
    ) {
        this.steerMotor = steerMotor;
        this.driveMotor = driveMotor;

        configureSteerMotor(steerConfig);
        configureDriveMotor();
    }

    private void configureSteerMotor(TalonFXConfiguration steerConfig) {
        steerMotor.applyConfiguration(steerConfig);
    }
    private void configureDriveMotor() {
        driveMotor.applyConfiguration(new TalonFXConfiguration());
    }

    protected SimpleMotorSimulation getSteerMotor() {
        return steerMotor;
    }
    protected SimpleMotorSimulation getDriveMotor() {
        return driveMotor;
    }

}
