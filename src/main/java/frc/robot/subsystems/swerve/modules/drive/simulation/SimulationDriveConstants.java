package frc.robot.subsystems.swerve.modules.drive.simulation;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.simulation.SimpleMotorSimulation;

public record SimulationDriveConstants(SimpleMotorSimulation driveMotor, boolean enableFOC) {

    public SimulationDriveConstants(DCMotorSim driveMotor, boolean enableFOC) {
        this(new SimpleMotorSimulation(driveMotor), enableFOC);
    }

}
