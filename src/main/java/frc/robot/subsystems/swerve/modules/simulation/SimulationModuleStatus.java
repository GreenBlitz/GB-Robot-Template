package frc.robot.subsystems.swerve.modules.simulation;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.simulation.SimpleMotorSimulation;

public class SimulationModuleStatus {

    private final SimpleMotorSimulation steerMotor, driveMotor;

    public SimulationModuleStatus(SimulationModuleConfigObject simulationModuleConfigObject) {
        this.driveMotor = simulationModuleConfigObject.getDriveMotor();
        this.steerMotor = simulationModuleConfigObject.getSteerMotor();
    }


    // Drive Motor Status
    public Rotation2d getDrivePositionAngle() {
        return driveMotor.getPosition();
    }

    public Rotation2d getDriveVelocityAnglePerSecond() {
        return driveMotor.getVelocity();
    }

    public double getDriveVoltage() {
        return driveMotor.getVoltage();
    }

    public double getDriveCurrent() {
        return driveMotor.getCurrent();
    }


    // Steer Motor Status
    public Rotation2d getSteerPosition() {
        return steerMotor.getPosition();
    }

    public Rotation2d getSteerVelocity() {
        return steerMotor.getVelocity();
    }

    public double getSteerVoltage() {
        return steerMotor.getVoltage();
    }

}
