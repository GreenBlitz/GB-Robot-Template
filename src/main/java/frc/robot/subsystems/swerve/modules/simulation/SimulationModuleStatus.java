package frc.robot.subsystems.swerve.modules.simulation;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.simulation.SimpleMotorSimulation;

class SimulationModuleStatus {

    private final SimpleMotorSimulation steerMotor, driveMotor;

    protected SimulationModuleStatus(SimpleMotorSimulation steerMotor, SimpleMotorSimulation driveMotor) {
        this.steerMotor = steerMotor;
        this.driveMotor = driveMotor;
    }


    protected Rotation2d getSteerPosition() {
        return steerMotor.getPosition();
    }

    protected Rotation2d getSteerVelocity() {
        return steerMotor.getVelocity();
    }

    protected double getSteerVoltage() {
        return steerMotor.getVoltage();
    }


    protected Rotation2d getDrivePositionAngle() {
        return driveMotor.getPosition();
    }

    protected Rotation2d getDriveVelocityAnglePerSecond() {
        return driveMotor.getVelocity();
    }

    protected double getDriveVoltage() {
        return driveMotor.getVoltage();
    }

    protected double getDriveCurrent() {
        return driveMotor.getCurrent();
    }

}
