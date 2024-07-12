package frc.robot.subsystems.swerve.modules.simulation;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.simulation.SimpleMotorSimulation;

public class SimulationModuleStatus {

    private final SimpleMotorSimulation steerMotor, driveMotor;

    public SimulationModuleStatus(SimpleMotorSimulation steerMotor, SimpleMotorSimulation driveMotor) {
        this.steerMotor = steerMotor;
        this.driveMotor = driveMotor;
    }


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
