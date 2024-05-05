package frc.robot.subsystems.swerve.modules.simulationmodule;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.modules.ModuleConstants;
import frc.robot.subsystems.swerve.modules.ModuleUtils;
import frc.robot.subsystems.swerve.modules.moduleinterface.IModule;
import frc.robot.subsystems.swerve.modules.moduleinterface.ModuleInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class SimulationModule implements IModule {

    private final ModuleUtils.ModuleName moduleName;

    private final SimulationModuleActions simulationModuleActions;

    private final SimulationModuleStatus simulationModuleStatus;

    public SimulationModule(ModuleUtils.ModuleName moduleName) {
        this.moduleName = moduleName;

        SimulationModuleConfigObject simulationModuleConfigObject = new SimulationModuleConfigObject();
        this.simulationModuleActions = new SimulationModuleActions(simulationModuleConfigObject);
        this.simulationModuleStatus = new SimulationModuleStatus(simulationModuleConfigObject);
    }

    @Override
    public void setTargetOpenLoopVelocity(double targetVelocityMetersPerSecond) {
        final double voltage = ModuleUtils.velocityToOpenLoopVoltage(
                targetVelocityMetersPerSecond,
                ModuleConstants.WHEEL_DIAMETER_METERS,
                simulationModuleStatus.getSteerVelocity(),
                0,
                ModuleConstants.MAX_SPEED_PER_SECOND,
                ModuleConstants.VOLTAGE_COMPENSATION_SATURATION
        );
        Logger.recordOutput(ModuleUtils.getLoggingPath(moduleName) + "driveMotorVoltage", simulationModuleStatus.getDriveVoltage());
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
    public void stop() {
        simulationModuleActions.stop();
    }

    @Override
    public void setBrake(boolean brake) {
        Logger.recordOutput(ModuleUtils.getLoggingPath(moduleName) + "driveMotorVoltage", simulationModuleStatus.getDriveVoltage());
    }

    @Override
    public void resetByEncoder() {
        Logger.recordOutput(ModuleUtils.getLoggingPath(moduleName) + "tried to reset by encoder");
    }


    @Override
    public void updateInputs(ModuleInputsAutoLogged inputs) {
        inputs.driveMotorDistance = simulationModuleStatus.getDrivePositionAngle();
        inputs.driveMotorVelocity = simulationModuleStatus.getDriveVelocityAnglePerSecond();
        inputs.driveMotorCurrent = simulationModuleStatus.getDriveCurrent();
        inputs.driveMotorVoltage = simulationModuleStatus.getDriveVoltage();

        inputs.steerMotorAngle = simulationModuleStatus.getSteerPosition();
        inputs.steerMotorVoltage = simulationModuleStatus.getSteerVoltage();

        inputs.odometryUpdatesDriveDistance = new Rotation2d[]{inputs.driveMotorDistance};
        inputs.odometryUpdatesSteerAngle = new Rotation2d[]{inputs.steerMotorAngle};
    }

}
