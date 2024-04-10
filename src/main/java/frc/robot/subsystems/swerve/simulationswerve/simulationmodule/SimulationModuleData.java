package frc.robot.subsystems.swerve.simulationswerve.simulationmodule;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.simulation.SimpleMotorSimulation;
import frc.robot.subsystems.swerve.ModuleUtils;

public class SimulationModuleData {

    private final SimpleMotorSimulation steerMotor, driveMotor;

    public SimulationModuleData(SimulationModuleConfigObject simulationModuleConfigObject) {
        this.driveMotor = simulationModuleConfigObject.getDriveMotor();
        this.steerMotor = simulationModuleConfigObject.getSteerMotor();
    }

    public double getSteerVoltage() {
        return steerMotor.getVoltage();
    }

    public Rotation2d getSteerVelocity() {
        return Rotation2d.fromRotations(steerMotor.getVelocityRevolutionsPerSecond());
    }

    public Rotation2d getSteerPosition() {
        return Rotation2d.fromRotations(steerMotor.getPositionRevolutions());
    }

    public double getDriveVoltage() {
        return driveMotor.getVoltage();
    }

    public double getDriveCurrent() {
        return driveMotor.getCurrent();
    }

    public double getDriveVelocityInMeters() {
        return ModuleUtils.toDriveDistance(driveMotor.getVelocityRevolutionsPerSecond());
    }

    public double getDrivePositionInMeters() {
        return ModuleUtils.toDriveDistance(driveMotor.getPositionRevolutions());
    }
}
