package frc.robot.subsystems.swerve.simulationswerve.simulationmodule;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.swerve.swerveinterface.*;
import org.littletonrobotics.junction.Logger;

public class SimulationModule implements IModule {

    private final ModuleUtils.ModuleName moduleName;

    private final SimulationModuleActions simulationModuleActions;
    private final SimulationModuleStatus simulationModuleData;

    public SimulationModule(ModuleUtils.ModuleName moduleName) {
        this.moduleName = moduleName;
        SimulationModuleConfigObject simulationModuleConfigObject = new SimulationModuleConfigObject();
        this.simulationModuleActions = new SimulationModuleActions(simulationModuleConfigObject);
        this.simulationModuleData = new SimulationModuleStatus(simulationModuleConfigObject);
    }

    @Override
    public void setTargetOpenLoopVelocity(double targetVelocityMetersPerSecond) {
        final double voltage = ModuleUtils.velocityToOpenLoopVoltage(
                targetVelocityMetersPerSecond,
                ModuleConstants.WHEEL_DIAMETER_METERS,
                simulationModuleData.getSteerVelocity().getRotations(),
                0,
                ModuleConstants.MAX_SPEED_REVOLUTIONS_PER_SECOND,
                ModuleConstants.VOLTAGE_COMPENSATION_SATURATION
        );
        Logger.recordOutput(ModuleUtils.getLoggingPath(moduleName) + "driveVoltage", simulationModuleData.getDriveVoltage());
        simulationModuleActions.setTargetOpenLoopVelocity(voltage);
    }

    @Override
    public void setTargetClosedLoopVelocity(double targetVelocityMetersPerSecond) {
        setTargetOpenLoopVelocity(targetVelocityMetersPerSecond);
    }

    @Override
    public void setTargetAngle(Rotation2d angle) {
        simulationModuleActions.setTargetAngle(angle);
    }

    @Override
    public void resetByEncoder() {
        Logger.recordOutput(ModuleUtils.getLoggingPath(moduleName) + "tried to reset by encoder");
    }

    @Override
    public void stop() {
        simulationModuleActions.stop();
    }

    @Override
    public void setBrake(boolean brake) {
        Logger.recordOutput(ModuleUtils.getLoggingPath(moduleName) + "driveVoltage", simulationModuleData.getDriveVoltage());
    }

    @Override
    public void updateInputs(ModuleInputsAutoLogged inputs) {
        inputs.steerAngleDegrees = simulationModuleData.getSteerPosition().getDegrees();
        inputs.odometryUpdatesSteerAngleDegrees = new double[]{inputs.steerAngleDegrees};
        inputs.steerVoltage = simulationModuleData.getSteerVoltage();

        inputs.driveDistanceMeters = simulationModuleData.getDrivePositionInMeters();
        inputs.odometryUpdatesDriveDistanceMeters = new double[]{inputs.driveDistanceMeters};
        inputs.driveVelocityMetersPerSecond = simulationModuleData.getDriveVelocityInMeters();
        inputs.driveCurrent = simulationModuleData.getDriveCurrent();
        inputs.driveVoltage = simulationModuleData.getDriveVoltage();
    }
}
