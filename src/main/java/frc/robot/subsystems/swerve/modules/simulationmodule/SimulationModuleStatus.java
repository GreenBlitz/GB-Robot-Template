package frc.robot.subsystems.swerve.modules.simulationmodule;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.simulation.SimpleMotorSimulation;
import frc.robot.subsystems.swerve.ModuleUtils;

public class SimulationModuleStatus {

    private final SimpleMotorSimulation steerMotor, driveMotor;

    public SimulationModuleStatus(SimulationModuleConfigObject simulationModuleConfigObject) {
        this.driveMotor = simulationModuleConfigObject.getDriveMotor();
        this.steerMotor = simulationModuleConfigObject.getSteerMotor();
    }

    public double getSteerVoltage() {
        return steerMotor.getVoltage();
    }

    public Rotation2d getSteerVelocity() {
        return steerMotor.getVelocity();
    }

    public Rotation2d getSteerPosition() {
        return steerMotor.getPosition();
    }

    public double getDriveVoltage() {
        return driveMotor.getVoltage();
    }

    public double getDriveCurrent() {
        return driveMotor.getCurrent();
    }

    public double getDriveVelocityInMeters() {
        return ModuleUtils.toDriveDistance(driveMotor.getVelocity().getRotations());
    }

    public double getDrivePositionInMeters() {
        return ModuleUtils.toDriveDistance(driveMotor.getPosition().getRotations());
    }

}
