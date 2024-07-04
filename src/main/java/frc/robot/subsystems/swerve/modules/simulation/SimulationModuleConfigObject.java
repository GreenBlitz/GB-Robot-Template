package frc.robot.subsystems.swerve.modules.simulation;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.simulation.SimpleMotorSimulation;
import frc.robot.subsystems.swerve.modules.ModuleConstants;

public class SimulationModuleConfigObject {

    private final SimpleMotorSimulation steerMotor, driveMotor;

    public SimulationModuleConfigObject() {
        this.steerMotor = new SimpleMotorSimulation(
                new DCMotorSim(
                        SimulationModuleConstants.STEER_MOTOR_GEARBOX,
                        ModuleConstants.STEER_GEAR_RATIO,
                        SimulationModuleConstants.STEER_MOMENT_OF_INERTIA
                )
        );
        this.driveMotor = new SimpleMotorSimulation(
                new DCMotorSim(
                        SimulationModuleConstants.DRIVE_MOTOR_GEARBOX,
                        ModuleConstants.DRIVE_GEAR_RATIO,
                        SimulationModuleConstants.DRIVE_MOMENT_OF_INERTIA
                )
        );

        configureDriveMotor();
        configureSteerMotor();
    }

    private void configureDriveMotor() {
        driveMotor.applyConfiguration(new TalonFXConfiguration());
    }

    private void configureSteerMotor() {
        steerMotor.applyConfiguration(SimulationModuleConstants.STEER_MOTOR_CONFIG);
    }

    public SimpleMotorSimulation getDriveMotor() {
        return driveMotor;
    }

    public SimpleMotorSimulation getSteerMotor() {
        return steerMotor;
    }

}
